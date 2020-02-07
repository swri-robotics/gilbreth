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
/*
 * Desc: Object disposal plugin
 * Author: Deanna Hood
 */
#ifndef _GAZEBO_OBJECT_DISPOSAL_PLUGIN_HH_
#define _GAZEBO_OBJECT_DISPOSAL_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <ignition/math/Pose3.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/util/system.hh>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/callback_queue.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <thread>
#include <queue>

#include "SideContactPlugin.hh"

template <typename T>
class ConcurrentQueue
{
public:
  ConcurrentQueue()
  {

  }

  ~ConcurrentQueue()
  {

  }

  void pop()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.pop_front();
  }

  T front()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.front();
  }

  bool empty()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty();
  }

  void push(const T& e)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push_back(e);
  }

  bool hasEntry(const T& e)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto ref = std::find(queue_.begin(),queue_.end(),e);
    return ref != queue_.end();
  }

  std::size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
  }

private:

  std::list<T> queue_;
  std::mutex mutex_;
};

namespace gazebo
{
  /// \brief A plugin for a contact sensor attached to a model disposal unit.
  class GAZEBO_VISIBLE ObjectDisposalPlugin : public SideContactPlugin
  {
    /// \brief Constructor.
    public: ObjectDisposalPlugin();

    /// \brief Destructor.
    public: virtual ~ObjectDisposalPlugin();

    /// \brief Load the model plugin.
    /// \param[in] _model Pointer to the model that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the world update event
    protected: void OnUpdate(const common::UpdateInfo &_info);

    /// \brief Act on models that are ontop of the sensor's link
    protected: void ActOnContactingModels();

    /**
     * @brief Callback that processes ROS related events
     */
    protected: void processROSQueue();

    /**
     * @brief Calls the service to delete models
     */
    void publishDeactivatedObjects();

    /// \brief If true, only delete models if their CoG is within the bounding box of the link
    protected: bool centerOfGravityCheck;

    /// \brief Pose where the object will be teleported.
    protected: ignition::math::Pose3d disposalPose; // TODO: Remove this seemingly unused member.

    protected:
    // ROS Connection
    std::shared_ptr<ros::NodeHandle> nh_;
    ros::CallbackQueue ros_queue_;
    std::thread ros_queue_thread_;
    ros::Publisher disposed_models_pub_;
    ConcurrentQueue<std::string> disposed_models_queue_;
    std::string world_frame_id_;

  };
}
#endif

