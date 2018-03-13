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

#include <limits>
#include <string>
#include <gazebo/transport/Node.hh>
#include <boost/format.hpp>
#include "gilbreth_gazebo/plugins/ObjectDisposalPlugin.hh"
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/ModelStates.h>

static const double ROS_QUEUE_TIMEOUT = 0.1;
static const std::string DISPOSED_MODELS_TOPIC = "gazebo/disposed_models";
static const std::string DEFAULT_WORLD_FRAME_ID = "world";
static const Eigen::Vector3d DISPOSAL_NOMINAL_LOCATION = Eigen::Vector3d(0,0,-10); // meters
static const double DISPOSAL_Y_INCREMENT = 1.25f;

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ObjectDisposalPlugin)

/////////////////////////////////////////////////
ObjectDisposalPlugin::ObjectDisposalPlugin() : SideContactPlugin()
{
}

/////////////////////////////////////////////////
ObjectDisposalPlugin::~ObjectDisposalPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);
  this->parentSensor.reset();
  this->world.reset();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  SideContactPlugin::Load(_model, _sdf);

  if (this->updateRate > 0)
    gzdbg << "ObjectDisposalPlugin running at " << this->updateRate << " Hz\n";
  else
    gzdbg << "ObjectDisposalPlugin running at the default update rate\n";

  this->centerOfGravityCheck = false;
  if (_sdf->HasElement("center_of_gravity_check"))
  {
    this->centerOfGravityCheck = _sdf->Get<bool>("center_of_gravity_check");
  }

  if (!_sdf->HasElement("disposal_pose"))
  {
    gzerr << "ObjectDisposalPlugin: Unable to find <disposal_pose> element\n";
    return;
  }

  this->disposalPose = _sdf->Get<math::Pose>("disposal_pose");

  // connecting to ROS
  GZ_ASSERT(ros::isInitialized(),"ROS is not initialized, can't subscribe to delete model server");

  // setting up ROS connections
  nh_.reset(new ros::NodeHandle());
  disposed_models_pub_ = nh_->advertise<gazebo_msgs::ModelStates>(DISPOSED_MODELS_TOPIC,1);

  // setting up info
  world_frame_id_ = DEFAULT_WORLD_FRAME_ID;
  gzmsg << boost::str(boost::format("World Frame ID '%1%'") % world_frame_id_)<< std::endl;

  // ROS Thread setup
  ros_queue_thread_ = std::thread(std::bind(&ObjectDisposalPlugin::processROSQueue,this));

}

void ObjectDisposalPlugin::processROSQueue()
{
  ros::Duration loop_pause(ROS_QUEUE_TIMEOUT);
  while(nh_->ok())
  {
    publishDeactivatedObjects();
    loop_pause.sleep();
  }
}

void ObjectDisposalPlugin::publishDeactivatedObjects()
{
  gazebo_msgs::DeleteModel delete_model_srv;
  gazebo_msgs::SetModelState set_state_srv;

  gazebo_msgs::ModelStates models_msg;
  while(!disposed_models_queue_.empty())
  {
    std::string model_name = disposed_models_queue_.front();
    geometry_msgs::Pose pose;
    geometry_msgs::Twist twist;

    tf::poseEigenToMsg(Eigen::Affine3d::Identity(),pose);
    tf::vectorEigenToMsg(Eigen::Vector3d::Zero(),twist.linear);
    tf::vectorEigenToMsg(Eigen::Vector3d::Zero(),twist.angular);

    models_msg.name.push_back(model_name);
    models_msg.pose.push_back(pose);
    models_msg.twist.push_back(twist);

    disposed_models_queue_.pop();
  }

  if(!models_msg.name.empty())
  {
    disposed_models_pub_.publish(models_msg);
  }
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::OnUpdate(const common::UpdateInfo &/*_info*/)
{
  // If we're using a custom update rate value we have to check if it's time to
  // update the plugin or not.
  if (!this->TimeToExecute())
    return;

  this->CalculateContactingModels();
  this->ActOnContactingModels();
}

/////////////////////////////////////////////////
void ObjectDisposalPlugin::ActOnContactingModels()
{
  // Only remove models if their center of gravity is "above" the link
  // TODO: make more general than just z axis
  auto linkBox = this->parentLink->GetBoundingBox();
  auto linkBoxMax = linkBox.max;
  auto linkBoxMin = linkBox.min;
  linkBoxMin.z = std::numeric_limits<double>::lowest();
  linkBoxMax.z = std::numeric_limits<double>::max();
  auto disposalBox = math::Box(linkBoxMin, linkBoxMax);


  for (auto model : this->contactingModels) {
    if (model) {
      bool removeModel = true;
      if (this->centerOfGravityCheck)
      {
        // Calculate the center of gravity of the model
        math::Vector3 modelCog = math::Vector3::Zero;
        double modelMass = 0.0;
        for (auto modelLink : model->GetLinks())
        {
          double linkMass = modelLink->GetInertial()->GetMass();
          modelCog += modelLink->GetWorldCoGPose().pos * linkMass;
          modelMass += linkMass;
        }
        if (modelMass > 0.0)
        {
          modelCog /= modelMass;
        }
        removeModel = disposalBox.Contains(modelCog);
      }
      if (removeModel)
      {
        // teleporting model away (deleting them causes race conditions)
        math::Pose modelDisposalPose = model->GetWorldPose();
        auto dl = DISPOSAL_NOMINAL_LOCATION;
        modelDisposalPose.pos = this->disposalPose.pos + math::Vector3(
            dl.x(),
            dl.y() + DISPOSAL_Y_INCREMENT * (disposed_models_queue_.size() + 1),
            dl.z());

        gzdbg << "[" << this->model->GetName() << "] Teleporting model: " << model->GetName() << "\n";
        model->SetAutoDisable(true);
        model->SetWorldPose(modelDisposalPose);

        // Adding object to queue
        if(!disposed_models_queue_.hasEntry(model->GetName() ))
        {
          disposed_models_queue_.push(model->GetName());
        }

      }
    }
  }
}
