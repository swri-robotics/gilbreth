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

static const double WAIT_SERVICE_PERIOD = 5.0f;
static const double ROS_QUEUE_TIMEOUT = 0.1;
static const std::string DELETE_MODEL_SERVICE = "gazebo/delete_model";
static const std::string SET_MODEL_STATE_SERVICE = "/gazebo/set_model_state";
static const std::string DEFAULT_WORLD_FRAME_ID = "world";
static const Eigen::Vector3d DISPOSE_LOCATION = Eigen::Vector3d(0,0,-10); // meters

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
  delete_model_client_ = nh_->serviceClient<gazebo_msgs::DeleteModel>(DELETE_MODEL_SERVICE);
  set_state_client_ = nh_->serviceClient<gazebo_msgs::SetModelState>(SET_MODEL_STATE_SERVICE);
  GZ_ASSERT(delete_model_client_.waitForExistence(ros::Duration(WAIT_SERVICE_PERIOD)),
            boost::str(boost::format("The service '%1%' was not found") % DELETE_MODEL_SERVICE).c_str());

  // setting up info
  world_frame_id_ = DEFAULT_WORLD_FRAME_ID;
  gzmsg << boost::str(boost::format("World Frame ID '%1%'") % world_frame_id_)<< std::endl;

  // ROS Thread setup
  ros_queue_thread_ = std::thread(std::bind(&ObjectDisposalPlugin::processROSQueue,this));

  gzmsg << boost::str(boost::format("Connected to service '%1%'") % DELETE_MODEL_SERVICE)<< std::endl;


}

void ObjectDisposalPlugin::processROSQueue()
{
  ros::Duration loop_pause(ROS_QUEUE_TIMEOUT);
  while(nh_->ok())
  {
    deleteQueuedObjects();
    loop_pause.sleep();
  }
}

void ObjectDisposalPlugin::deleteQueuedObjects()
{
  gazebo_msgs::DeleteModel delete_model_srv;
  gazebo_msgs::SetModelState set_state_srv;
  while(!delete_model_queue_.empty())
  {
    std::string model_name = delete_model_queue_.front();

    // moving model out of the way
    gazebo_msgs::SetModelStateRequest& set_st_req = set_state_srv.request;
    set_st_req.model_state.model_name = model_name;
    Eigen::Affine3d pose = Eigen::Affine3d::Identity() * Eigen::Translation3d(DISPOSE_LOCATION);
    tf::poseEigenToMsg(pose,set_st_req.model_state.pose);
    set_st_req.model_state.reference_frame = world_frame_id_;

    if(!set_state_client_.call(set_state_srv))
    {
      gzwarn<<boost::str(boost::format("Failed to move model '%1%' to discard location")% model_name)<<std::endl;
    }

    delete_model_srv.request.model_name = model_name;
    if(delete_model_client_.call(delete_model_srv) && delete_model_srv.response.success)
    {
      gzdbg <<"Model '"<< delete_model_srv.request.model_name <<"' deleted"<<std::endl;
    }
    else
    {
      gzerr <<"Model deletion failed for '"<< delete_model_srv.request.model_name <<"'" <<std::endl;
    }

    delete_model_queue_.pop();
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
        // Queuing object for deletion
        if(!delete_model_queue_.hasEntry(model->GetName() ))
        {
          gzdbg<<boost::str(boost::format("Adding %1% to deletion queue") %model->GetName() )<<std::endl;
          delete_model_queue_.push(model->GetName());
        }

      }
    }
  }
}
