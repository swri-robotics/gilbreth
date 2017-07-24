/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <cstdlib>
#include <string>

#include <osrf_gear/TrayContents.h>

#include "ROSAriacKitTrayPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(KitTrayPlugin)

/////////////////////////////////////////////////
KitTrayPlugin::KitTrayPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
KitTrayPlugin::~KitTrayPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void KitTrayPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (_sdf->HasElement("faulty_parts"))
  {
    this->faultyPartNames.clear();
    sdf::ElementPtr faultyPartNamesElem = _sdf->GetElement("faulty_parts");
    if (faultyPartNamesElem->HasElement("name"))
    {
      sdf::ElementPtr faultyPartElem = faultyPartNamesElem->GetElement("name");
      while (faultyPartElem)
      {
        std::string faultyPartName = faultyPartElem->Get<std::string>();

        ROS_DEBUG_STREAM("Ignoring part: " << faultyPartName);
        this->faultyPartNames.push_back(faultyPartName);
        faultyPartElem = faultyPartElem->GetNextElement("name");
      }
    }
  }

  if (this->updateRate > 0)
    gzdbg << "KitTrayPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "KitTrayPlugin running at the default update rate\n";

  this->trayID = this->parentLink->GetScopedName();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosNode = new ros::NodeHandle("");
  this->currentKitPub = this->rosNode->advertise<osrf_gear::TrayContents>(
    "/ariac/trays", 1000, boost::bind(&KitTrayPlugin::OnSubscriberConnect, this, _1));
  this->publishingEnabled = true;

  // ROS service for clearing the tray
  std::string clearServiceName = "clear";
  if (_sdf->HasElement("clear_tray_service_name"))
    clearServiceName = _sdf->Get<std::string>("clear_tray_service_name");
  this->clearTrayServer =
    this->rosNode->advertiseService(clearServiceName, &KitTrayPlugin::HandleClearService, this);

  // Initialize Gazebo transport
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();

  // Gazebo subscription for the lock trays topic
  std::string lockModelsServiceName = "lock_models";
  if (_sdf->HasElement("lock_models_service_name"))
    lockModelsServiceName = _sdf->Get<std::string>("lock_models_service_name");
  this->lockModelsSub = this->gzNode->Subscribe(
    lockModelsServiceName, &KitTrayPlugin::HandleLockModelsRequest, this);
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
  {
    return;
  }

  if (!this->newMsg)
  {
    return;
  }

  auto prevNumberContactingModels = this->contactingModels.size();
  this->CalculateContactingModels();
  if (prevNumberContactingModels != this->contactingModels.size()) {
    ROS_DEBUG_STREAM(this->parentLink->GetScopedName() << ": number of contacting models: "
      << this->contactingModels.size());
  }
  this->ProcessContactingModels();
  if (this->publishingEnabled)
  {
    this->PublishKitMsg();
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::ProcessContactingModels()
{
  // Make sure that models fixed to the tray are included in the contacting models,
  // even if they aren't contacting the tray anymore.
  for (auto fixedJoint : this->fixedJoints)
  {
    auto link = fixedJoint->GetChild();
    this->contactingLinks.insert(link);
    this->contactingModels.insert(link->GetParentModel());
  }
  this->currentKit.objects.clear();
  auto trayPose = this->parentLink->GetWorldPose().Ign();
  for (auto model : this->contactingModels) {
    if (model) {
      model->SetAutoDisable(false);
      ariac::KitObject object;

      // Determine the object type
      object.type = ariac::DetermineModelType(model->GetName());

      // Determine if the object is faulty
      auto modelName = ariac::TrimNamespace(model->GetName());
      auto it = std::find(this->faultyPartNames.begin(), this->faultyPartNames.end(), modelName);
      object.isFaulty = it != this->faultyPartNames.end();

      // Determine the pose of the object in the frame of the tray
      math::Pose objectPose = model->GetWorldPose();
      ignition::math::Matrix4d transMat(trayPose);
      ignition::math::Matrix4d objectPoseMat(objectPose.Ign());
      object.pose = (transMat.Inverse() * objectPoseMat).Pose();
      object.pose.rot.Normalize();

      this->currentKit.objects.push_back(object);
    }
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::OnSubscriberConnect(const ros::SingleSubscriberPublisher& pub)
{
  auto subscriberName = pub.getSubscriberName();
  gzdbg << this->trayID << ": New subscription from node: " << subscriberName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && subscriberName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so subscribing to this topic is not permitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    // Disable publishing of kit messages.
    // This will break the scoring but ensure competitors can't cheat.
    this->publishingEnabled = false;
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::PublishKitMsg()
{
  // Publish current kit
  osrf_gear::TrayContents kitTrayMsg;
  kitTrayMsg.tray = this->trayID;
  for (const auto &obj : this->currentKit.objects)
  {
    osrf_gear::DetectedObject msgObj;
    msgObj.type = obj.type;
    msgObj.is_faulty = obj.isFaulty;
    msgObj.pose.position.x = obj.pose.pos.x;
    msgObj.pose.position.y = obj.pose.pos.y;
    msgObj.pose.position.z = obj.pose.pos.z;
    msgObj.pose.orientation.x = obj.pose.rot.x;
    msgObj.pose.orientation.y = obj.pose.rot.y;
    msgObj.pose.orientation.z = obj.pose.rot.z;
    msgObj.pose.orientation.w = obj.pose.rot.w;

    // Add the object to the kit.
    kitTrayMsg.objects.push_back(msgObj);
  }
  this->currentKitPub.publish(kitTrayMsg);
}

/////////////////////////////////////////////////
void KitTrayPlugin::UnlockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  for (auto fixedJoint : this->fixedJoints)
  {
    fixedJoint->Detach();
  }
  this->fixedJoints.clear();
}

/////////////////////////////////////////////////
void KitTrayPlugin::LockContactingModels()
{
  boost::mutex::scoped_lock lock(this->mutex);
  physics::JointPtr fixedJoint;
  gzdbg << "Number of models in contact with the tray: " << this->contactingModels.size() << std::endl;
  for (auto model : this->contactingModels)
  {
  // Create the joint that will attach the models
  fixedJoint = this->world->GetPhysicsEngine()->CreateJoint(
        "fixed", this->model);
  auto jointName = this->model->GetName() + "_" + model->GetName() + "__joint__";
  gzdbg << "Creating fixed joint: " << jointName << std::endl;
  fixedJoint->SetName(jointName);

  model->SetGravityMode(false);

  // Lift the part slightly because it will fall through the tray if the tray is animated
  model->SetWorldPose(model->GetWorldPose() + math::Pose(0,0,0.01,0,0,0));

  auto modelName = model->GetName();
  auto linkName = modelName + "::link";
  auto link = model->GetLink(linkName);
  if (link == NULL)
  {
    // If the model was inserted into the world using the "population" SDF tag,
    // the link will have an additional namespace of the model type.
    linkName = modelName + "::" + ariac::DetermineModelType(modelName) + "::link";
    link = model->GetLink(linkName);
    if (link == NULL)
    {
      gzwarn << "Couldn't find link to make joint with: " << linkName;
      continue;
    }
  }
  link->SetGravityMode(false);
  fixedJoint->Load(link, this->parentLink, math::Pose());
  fixedJoint->Attach(this->parentLink, link);
  fixedJoint->Init();
  this->fixedJoints.push_back(fixedJoint);
  model->SetAutoDisable(true);
  }
}

/////////////////////////////////////////////////
void KitTrayPlugin::HandleLockModelsRequest(ConstGzStringPtr &_msg)
{
  gzdbg << this->trayID << ": Handle clear tray service called.\n";
  (void)_msg;
  this->LockContactingModels();
}

/////////////////////////////////////////////////
bool KitTrayPlugin::HandleClearService(
  ros::ServiceEvent<std_srvs::Trigger::Request, std_srvs::Trigger::Response>& event)
{
  std_srvs::Trigger::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << this->trayID << ": Handle clear tray service called by: " << callerName << std::endl;

  // During the competition, this environment variable will be set.
  auto compRunning = std::getenv("ARIAC_COMPETITION");
  if (compRunning && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition is running so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  this->UnlockContactingModels();
  this->ClearContactingModels();
  res.success = true;
  return true;
}
