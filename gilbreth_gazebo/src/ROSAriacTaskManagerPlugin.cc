/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <mutex>
#include <ostream>
#include <string>
#include <vector>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Console.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/math/Pose.hh>
#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <sdf/sdf.hh>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>


#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/ROSAriacTaskManagerPlugin.hh"
#include "osrf_gear/AriacScorer.h"
#include <osrf_gear/ConveyorBeltControl.h>
#include <osrf_gear/ConveyorBeltState.h>
#include "osrf_gear/Kit.h"
#include "osrf_gear/KitObject.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/VacuumGripperState.h"

namespace gazebo
{
  /// \internal
  /// \brief Private data for the ROSAriacTaskManagerPlugin class.
  struct ROSAriacTaskManagerPluginPrivate
  {
    /// \brief World pointer.
    public: physics::WorldPtr world;

    /// \brief SDF pointer.
    public: sdf::ElementPtr sdf;

    /// \brief Collection of orders to announce.
    public: std::vector<ariac::Order> ordersToAnnounce;

    /// \brief Collection of orders which have been announced but are not yet complete.
    /// The order at the top of the stack is the active order.
    public: std::stack<ariac::Order> ordersInProgress;

    /// \brief Mapping between material types and their locations.
    public: std::map<std::string, std::vector<std::string> > materialLocations;

    /// \brief A scorer to mange the game score.
    public: AriacScorer ariacScorer;

    /// \brief The current game score.
    public: ariac::GameScore currentGameScore;

    /// \brief ROS node handle.
    public: std::unique_ptr<ros::NodeHandle> rosnode;

    /// \brief Publishes an order.
    public: ros::Publisher orderPub;

    /// \brief ROS subscriber for the tray states.
    public: ros::Subscriber trayInfoSub;

    /// \brief ROS subscriber for the gripper state.
    public: ros::Subscriber gripperStateSub;

    /// \brief Publishes the Gazebo task state.
    public: ros::Publisher taskStatePub;

    /// \brief Publishes the game score total.
    public: ros::Publisher taskScorePub;

    /// \brief Service that allows the user to start the competition.
    public: ros::ServiceServer compStartServiceServer;

    /// \brief Service that allows the user to end the competition.
    public: ros::ServiceServer compEndServiceServer;

    /// \brief Service that allows users to query the location of materials.
    public: ros::ServiceServer getMaterialLocationsServiceServer;

    /// \brief Service that allows a tray to be submitted for inspection.
    public: ros::ServiceServer submitTrayServiceServer;

    /// \brief Transportation node.
    public: transport::NodePtr node;

    /// \brief Publisher for enabling the object population on the conveyor.
    public: transport::PublisherPtr populatePub;

    /// \brief Client for controlling the conveyor.
    public: ros::ServiceClient conveyorControlClient;

    /// \brief Timer for regularly publishing state/score.
    public: ros::Timer statusPubTimer;

    /// \brief Connection event.
    public: event::ConnectionPtr connection;

    /// \brief Publish Gazebo server control messages.
    public: transport::PublisherPtr serverControlPub;

    /// \brief The time the last update was called.
    public: common::Time lastUpdateTime;

    /// \brief The time the sim time was last published.
    public: common::Time lastSimTimePublish;

    /// \brief The time specified in the object is relative to this time.
    public: common::Time gameStartTime;

    /// \brief The time in seconds permitted to complete the trial.
    public: double timeLimit;

    /// \brief The time in seconds that has been spent on the current order.
    public: double timeSpentOnCurrentOrder;

    /// \brief Pointer to the current state.
    public: std::string currentState = "init";

    /// \brief A mutex to protect currentState.
    public: std::mutex mutex;

    // During the competition, this environment variable will be set.
    bool competitonMode = false;
  };
}

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(ROSAriacTaskManagerPlugin)

/////////////////////////////////////////////////
static void fillOrderMsg(const ariac::Order &_order,
                        osrf_gear::Order &_msgOrder)
{
  _msgOrder.order_id = _order.orderID;
  for (const auto &kit : _order.kits)
  {
    osrf_gear::Kit msgKit;
    msgKit.kit_type = kit.kitType;
    for (const auto &obj : kit.objects)
    {
      osrf_gear::KitObject msgObj;
      msgObj.type = obj.type;
      msgObj.pose.position.x = obj.pose.pos.x;
      msgObj.pose.position.y = obj.pose.pos.y;
      msgObj.pose.position.z = obj.pose.pos.z;
      msgObj.pose.orientation.x = obj.pose.rot.x;
      msgObj.pose.orientation.y = obj.pose.rot.y;
      msgObj.pose.orientation.z = obj.pose.rot.z;
      msgObj.pose.orientation.w = obj.pose.rot.w;

      // Add the object to the kit.
      msgKit.objects.push_back(msgObj);
    }
    _msgOrder.kits.push_back(msgKit);
  }
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::ROSAriacTaskManagerPlugin()
  : dataPtr(new ROSAriacTaskManagerPluginPrivate)
{
}

/////////////////////////////////////////////////
ROSAriacTaskManagerPlugin::~ROSAriacTaskManagerPlugin()
{
  this->dataPtr->rosnode->shutdown();
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::Load(physics::WorldPtr _world,
  sdf::ElementPtr _sdf)
{
  gzdbg << "ARIAC VERSION: 1.1.4\n";
  auto competitionEnv = std::getenv("ARIAC_COMPETITION");
  this->dataPtr->competitonMode = competitionEnv != NULL;
  gzdbg << "ARIAC COMPETITION MODE: " << (this->dataPtr->competitonMode ? competitionEnv : "false") << std::endl;

  GZ_ASSERT(_world, "ROSAriacTaskManagerPlugin world pointer is NULL");
  GZ_ASSERT(_sdf, "ROSAriacTaskManagerPlugin sdf pointer is NULL");
  this->dataPtr->world = _world;
  this->dataPtr->sdf = _sdf;

  std::string robotNamespace = "";
  if (_sdf->HasElement("robot_namespace"))
  {
    robotNamespace = _sdf->GetElement(
      "robot_namespace")->Get<std::string>() + "/";
  }

  this->dataPtr->timeLimit = -1.0;
  if (_sdf->HasElement("competition_time_limit"))
    this->dataPtr->timeLimit = _sdf->Get<double>("competition_time_limit");

  std::string compEndServiceName = "end_competition";
  if (_sdf->HasElement("end_competition_service_name"))
    compEndServiceName = _sdf->Get<std::string>("end_competition_service_name");

  std::string compStartServiceName = "start_competition";
  if (_sdf->HasElement("start_competition_service_name"))
    compStartServiceName = _sdf->Get<std::string>("start_competition_service_name");

  std::string taskStateTopic = "competition_state";
  if (_sdf->HasElement("task_state_topic"))
    taskStateTopic = _sdf->Get<std::string>("task_state_topic");

  std::string taskScoreTopic = "current_score";
  if (_sdf->HasElement("task_score_topic"))
    taskScoreTopic = _sdf->Get<std::string>("task_score_topic");

  std::string conveyorControlTopic = "conveyor/control";
  if (_sdf->HasElement("conveyor_control_topic"))
    conveyorControlTopic = _sdf->Get<std::string>("conveyor_control_topic");

  std::string populationActivateTopic = "populate_belt";
  if (_sdf->HasElement("population_activate_topic"))
    populationActivateTopic = _sdf->Get<std::string>("population_activate_topic");

  std::string ordersTopic = "orders";
  if (_sdf->HasElement("orders_topic"))
    ordersTopic = _sdf->Get<std::string>("orders_topic");

  std::string submitTrayServiceName = "submit_tray";
  if (_sdf->HasElement("submit_tray_service_name"))
    submitTrayServiceName = _sdf->Get<std::string>("submit_tray_service_name");

  std::string getMaterialLocationsServiceName = "material_locations";
  if (_sdf->HasElement("material_locations_service_name"))
    getMaterialLocationsServiceName = _sdf->Get<std::string>("material_locations_service_name");


  // Parse the orders.
  sdf::ElementPtr orderElem = NULL;
  if (_sdf->HasElement("order"))
  {
    orderElem = _sdf->GetElement("order");
  }

  unsigned int orderCount = 0;
  while (orderElem)
  {
    // Parse the start time.
    double startTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("start_time"))
    {
      sdf::ElementPtr startTimeElement = orderElem->GetElement("start_time");
      startTime = startTimeElement->Get<double>();
    }

    // Parse the interruption criteria.
    int interruptOnUnwantedParts = -1;
    if (orderElem->HasElement("interrupt_on_unwanted_parts"))
    {
      sdf::ElementPtr interruptOnUnwantedPartsElem = orderElem->GetElement("interrupt_on_unwanted_parts");
      interruptOnUnwantedParts = interruptOnUnwantedPartsElem->Get<int>();
    }
    int interruptOnWantedParts = -1;
    if (orderElem->HasElement("interrupt_on_wanted_parts"))
    {
      sdf::ElementPtr interruptOnWantedPartsElem = orderElem->GetElement("interrupt_on_wanted_parts");
      interruptOnWantedParts = interruptOnWantedPartsElem->Get<int>();
    }

    // Parse the allowed completion time.
    double allowedTime = std::numeric_limits<double>::infinity();
    if (orderElem->HasElement("allowed_time"))
    {
      sdf::ElementPtr allowedTimeElement = orderElem->GetElement("allowed_time");
      allowedTime = allowedTimeElement->Get<double>();
    }

    // Parse the kits.
    if (!orderElem->HasElement("kit"))
    {
      gzerr << "Unable to find <kit> element in <order>. Ignoring" << std::endl;
      orderElem = orderElem->GetNextElement("order");
      continue;
    }

    // Store all kits for an order.
    std::vector<ariac::Kit> kits;

    sdf::ElementPtr kitElem = orderElem->GetElement("kit");
    while (kitElem)
    {
      // Check the validity of the kit.
      if (!kitElem->HasElement("object"))
      {
        gzerr << "Unable to find <object> element in <kit>. Ignoring"
              << std::endl;
        kitElem = kitElem->GetNextElement("kit");
        continue;
      }

      ariac::Kit kit;

      // Parse the kit type.
      ariac::KitType_t kitType;
      if (kitElem->HasElement("kit_type"))
      {
        kitType = kitElem->Get<std::string>("kit_type");
      }
      kit.kitType = kitType;

      // Parse the objects inside the kit.
      sdf::ElementPtr objectElem = kitElem->GetElement("object");
      while (objectElem)
      {
        // Parse the object type.
        if (!objectElem->HasElement("type"))
        {
          gzerr << "Unable to find <type> in object.\n";
          objectElem = objectElem->GetNextElement("object");
          continue;
        }
        sdf::ElementPtr typeElement = objectElem->GetElement("type");
        std::string type = typeElement->Get<std::string>();

        // Parse the object pose (optional).
        if (!objectElem->HasElement("pose"))
        {
          gzerr << "Unable to find <pose> in object.\n";
          objectElem = objectElem->GetNextElement("object");
          continue;
        }
        sdf::ElementPtr poseElement = objectElem->GetElement("pose");
        math::Pose pose = poseElement->Get<math::Pose>();

        // Add the object to the kit.
        bool isFaulty = false;  // We never want to request faulty parts.
        ariac::KitObject obj = {type, isFaulty, pose};
        kit.objects.push_back(obj);

        objectElem = objectElem->GetNextElement("object");
      }

      // Add a new kit to the collection of kits.
      kits.push_back(kit);

      kitElem = kitElem->GetNextElement("kit");
    }

    // Add a new order.
    ariac::OrderID_t orderID = "order_" + std::to_string(orderCount++);
    ariac::Order order = {orderID, startTime, interruptOnUnwantedParts, interruptOnWantedParts, allowedTime, kits, 0.0};
    this->dataPtr->ordersToAnnounce.push_back(order);

    orderElem = orderElem->GetNextElement("order");
  }

  // Sort the orders by their start times.
  std::sort(this->dataPtr->ordersToAnnounce.begin(), this->dataPtr->ordersToAnnounce.end());

  // Debug output.
  // gzdbg << "Orders:" << std::endl;
  // for (auto order : this->dataPtr->ordersToAnnounce)
  //   gzdbg << order << std::endl;

  // Parse the material storage locations.
  if (_sdf->HasElement("material_locations"))
  {
    sdf::ElementPtr materialLocationsElem = _sdf->GetElement("material_locations");
    sdf::ElementPtr materialElem = NULL;
    if (materialLocationsElem->HasElement("material"))
    {
      materialElem = materialLocationsElem->GetElement("material");
    }
    while (materialElem)
    {
      std::string materialType = materialElem->Get<std::string>("type");
      std::vector<std::string> locations;

      // Parse locations of this material.
      sdf::ElementPtr locationElem = NULL;
      if (materialElem->HasElement("location"))
      {
        locationElem = materialElem->GetElement("location");
      }
      while (locationElem)
      {
        std::string location = locationElem->Get<std::string>("storage_unit");
        locations.push_back(location);
        locationElem = locationElem->GetNextElement("location");
      }
      this->dataPtr->materialLocations[materialType] = locations;
      materialElem = materialElem->GetNextElement("material");
    }
  }

  // Initialize ROS
  this->dataPtr->rosnode.reset(new ros::NodeHandle(robotNamespace));

  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  // Publisher for announcing new orders.
  this->dataPtr->orderPub = this->dataPtr->rosnode->advertise<
    osrf_gear::Order>(ordersTopic, 1000, true);  // latched=true

  // Publisher for announcing new state of the competition.
  this->dataPtr->taskStatePub = this->dataPtr->rosnode->advertise<
    std_msgs::String>(taskStateTopic, 1000);

  // Publisher for announcing the score of the game.
  this->dataPtr->taskScorePub = this->dataPtr->rosnode->advertise<
    std_msgs::Float32>(taskScoreTopic, 1000);

  // Service for starting the competition.
  this->dataPtr->compStartServiceServer =
    this->dataPtr->rosnode->advertiseService(compStartServiceName,
      &ROSAriacTaskManagerPlugin::HandleStartService, this);

  // Service for ending the competition.
  this->dataPtr->compEndServiceServer =
    this->dataPtr->rosnode->advertiseService(compEndServiceName,
      &ROSAriacTaskManagerPlugin::HandleEndService, this);

  // Service for submitting trays for inspection.
  this->dataPtr->submitTrayServiceServer =
    this->dataPtr->rosnode->advertiseService(submitTrayServiceName,
      &ROSAriacTaskManagerPlugin::HandleSubmitTrayService, this);

  // Service for querying material storage locations.
  this->dataPtr->getMaterialLocationsServiceServer =
    this->dataPtr->rosnode->advertiseService(getMaterialLocationsServiceName,
      &ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService, this);

  // Client for the conveyor control commands.
  this->dataPtr->conveyorControlClient =
    this->dataPtr->rosnode->serviceClient<osrf_gear::ConveyorBeltControl>(
      conveyorControlTopic);

  // Timer for regularly publishing state/score.
  this->dataPtr->statusPubTimer =
    this->dataPtr->rosnode->createTimer(ros::Duration(0.1),
      &ROSAriacTaskManagerPlugin::PublishStatus, this);

  // Initialize Gazebo transport.
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->populatePub =
    this->dataPtr->node->Advertise<msgs::GzString>(populationActivateTopic);

  // Initialize the game scorer.
  this->dataPtr->trayInfoSub = this->dataPtr->rosnode->subscribe(
    "/ariac/trays", 10, &AriacScorer::OnTrayInfoReceived, &this->dataPtr->ariacScorer);
  this->dataPtr->gripperStateSub = this->dataPtr->rosnode->subscribe(
    "/ariac/gripper/state", 10, &AriacScorer::OnGripperStateReceived,
    &this->dataPtr->ariacScorer);

  this->dataPtr->serverControlPub =
    this->dataPtr->node->Advertise<msgs::ServerControl>("/gazebo/server/control");

  this->dataPtr->connection = event::Events::ConnectWorldUpdateEnd(
    boost::bind(&ROSAriacTaskManagerPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::OnUpdate()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto currentSimTime = this->dataPtr->world->GetSimTime();
  if ((currentSimTime - this->dataPtr->lastSimTimePublish).Double() >= 1.0)
  {
    gzdbg << "Sim time: " << currentSimTime.Double() << std::endl;
    this->dataPtr->lastSimTimePublish = currentSimTime;
  }

  double elapsedTime = (currentSimTime - this->dataPtr->lastUpdateTime).Double();
  if (this->dataPtr->timeLimit >= 0 && this->dataPtr->currentState == "go" &&
    (currentSimTime - this->dataPtr->gameStartTime) > this->dataPtr->timeLimit)
  {
    this->dataPtr->currentState = "end_game";
  }

  if (this->dataPtr->currentState == "ready")
  {
    this->dataPtr->gameStartTime = currentSimTime;
    this->dataPtr->currentState = "go";

    // TODO(dhood): only start the belt if there are belt parts specified
    this->ControlConveyorBelt(100);
    this->PopulateConveyorBelt();
  }
  else if (this->dataPtr->currentState == "go")
  {

    // Update the order manager.
    this->ProcessOrdersToAnnounce();

    // Update the score.
    this->dataPtr->ariacScorer.Update(elapsedTime);
    auto gameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (gameScore.total() != this->dataPtr->currentGameScore.total())
    {
      std::ostringstream logMessage;
      logMessage << "Current game score: " << gameScore.total();
      ROS_DEBUG_STREAM(logMessage.str().c_str());
      gzdbg << logMessage.str() << std::endl;
      this->dataPtr->currentGameScore = gameScore;
    }

    if (!this->dataPtr->ordersInProgress.empty())
    {
      this->dataPtr->ordersInProgress.top().timeTaken += elapsedTime;
      auto orderID = this->dataPtr->ordersInProgress.top().orderID;
      // TODO: timing should probably be managed by the scorer but we want to use sim time
      this->dataPtr->timeSpentOnCurrentOrder = this->dataPtr->ordersInProgress.top().timeTaken;

      // Check for completed orders.
      bool orderCompleted = this->dataPtr->ariacScorer.IsOrderComplete(orderID);
      if (orderCompleted)
      {
        std::ostringstream logMessage;
        logMessage << "Order complete: " << orderID;
        ROS_ERROR_STREAM(("[INFO] " + logMessage.str()).c_str());
        gzdbg << logMessage.str() << std::endl;
        this->StopCurrentOrder();
      }
      else
      {
        // Check if the time limit for the current order has been exceeded.
        if (this->dataPtr->timeSpentOnCurrentOrder > this->dataPtr->ordersInProgress.top().allowedTime)
        {
          std::ostringstream logMessage;
          logMessage << "Order timed out: " << orderID;
          ROS_ERROR_STREAM(("[INFO] " + logMessage.str()).c_str());
          gzdbg << logMessage.str() << std::endl;
          this->StopCurrentOrder();
        }
      }
    }

    if (this->dataPtr->ordersInProgress.empty() && this->dataPtr->ordersToAnnounce.empty())
    {
      gzdbg << "No more orders to process." << std::endl;
      this->dataPtr->currentState = "end_game";
    }
  }
  else if (this->dataPtr->currentState == "end_game")
  {
    this->dataPtr->currentGameScore = this->dataPtr->ariacScorer.GetGameScore();
    if (this->dataPtr->gameStartTime != common::Time())
    {
      this->dataPtr->currentGameScore.totalProcessTime =
        (currentSimTime - this->dataPtr->gameStartTime).Double();
    }
    std::ostringstream logMessage;
    logMessage << "End of trial. Final score: " << \
      this->dataPtr->currentGameScore.total() << "\nScore breakdown:\n" << \
      this->dataPtr->currentGameScore;
    ROS_ERROR_STREAM(("[INFO] " + logMessage.str()).c_str());
    gzdbg << logMessage.str() << std::endl;
    this->dataPtr->currentState = "done";

    auto v = std::getenv("ARIAC_EXIT_ON_COMPLETION");
    if (v)
    {
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->dataPtr->serverControlPub->Publish(msg);
      gazebo::shutdown();
    }
  }

  this->dataPtr->lastUpdateTime = currentSimTime;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PublishStatus(const ros::TimerEvent&)
{
  std_msgs::Float32 scoreMsg;
  scoreMsg.data = this->dataPtr->currentGameScore.total();
  this->dataPtr->taskScorePub.publish(scoreMsg);

  std_msgs::String stateMsg;
  stateMsg.data = this->dataPtr->currentState;
  this->dataPtr->taskStatePub.publish(stateMsg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ProcessOrdersToAnnounce()
{
  if (this->dataPtr->ordersToAnnounce.empty())
    return;

  auto nextOrder = this->dataPtr->ordersToAnnounce.front();
  bool interruptOnUnwantedParts = nextOrder.interruptOnUnwantedParts > 0;
  bool interruptOnWantedParts = nextOrder.interruptOnWantedParts > 0;
  bool noActiveOrder = this->dataPtr->ordersInProgress.empty();
  auto elapsed = this->dataPtr->world->GetSimTime() - this->dataPtr->gameStartTime;
  bool announceNextOrder = false;

  // Check whether announce a new order from the list.
  // Announce next order if the appropriate amount of time has elapsed
  announceNextOrder |= elapsed.Double() >= nextOrder.startTime;
  // Announce next order if there is no active order and we are waiting to interrupt
  announceNextOrder |= noActiveOrder && (interruptOnWantedParts || interruptOnUnwantedParts);

  int maxNumUnwantedParts = 0;
  int maxNumWantedParts = 0;
  // Check if it's time to interrupt (skip if we're already interrupting anyway)
  if (!announceNextOrder && (interruptOnWantedParts || interruptOnUnwantedParts))
  {
    // Check if the parts in the trays are enough to interrupt the current order

    // Determine what parts are in the next order
    std::vector<std::string> partsInNextOrder;
    for (const auto & kit : nextOrder.kits)
    {
      for (const auto & part : kit.objects)
      {
        partsInNextOrder.push_back(part.type);
      }
    }

    // Check the trays for parts from the pending order
    std::vector<int> numUnwantedPartsOnTrays;
    std::vector<int> numWantedPartsOnTrays;
    for (const auto & tray : this->dataPtr->ariacScorer.GetTrays())
    {
      int numUnwantedPartsOnTray = 0;
      int numWantedPartsOnTray = 0;
      std::vector<std::string> partsInNextOrder_copy(partsInNextOrder);
      for (const auto part : tray.currentKit.objects)
      {
        // Don't count faulty parts, because they have to be removed anyway.
        if (part.isFaulty)
        {
          continue;
        }
        auto it = std::find(partsInNextOrder_copy.begin(), partsInNextOrder_copy.end(), part.type);
        if (it == partsInNextOrder_copy.end())
        {
          numUnwantedPartsOnTray += 1;
        }
        else
        {
          numWantedPartsOnTray += 1;
          partsInNextOrder_copy.erase(it);
        }
      }
      numUnwantedPartsOnTrays.push_back(numUnwantedPartsOnTray);
      numWantedPartsOnTrays.push_back(numWantedPartsOnTray);
    }
    maxNumUnwantedParts = *std::max_element(numUnwantedPartsOnTrays.begin(), numUnwantedPartsOnTrays.end());
    maxNumWantedParts = *std::max_element(numWantedPartsOnTrays.begin(), numWantedPartsOnTrays.end());

    // Announce next order if the appropriate number of wanted/unwanted parts are detected
    announceNextOrder |= interruptOnWantedParts && (maxNumWantedParts >= nextOrder.interruptOnWantedParts);
    announceNextOrder |= interruptOnUnwantedParts && (maxNumUnwantedParts >= nextOrder.interruptOnUnwantedParts);
  }

  if (announceNextOrder)
  {
    gzdbg << "New order to announce: " << nextOrder.orderID << std::endl;

    // Move order to the 'in process' stack
    this->dataPtr->ordersInProgress.push(ariac::Order(nextOrder));
    this->dataPtr->ordersToAnnounce.erase(this->dataPtr->ordersToAnnounce.begin());

    this->AssignOrder(nextOrder);
  }
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleStartService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  gzdbg << "Handle start service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  if (this->dataPtr->currentState == "init") {
    this->dataPtr->currentState = "ready";
    res.success = true;
    res.message = "competition started successfully!";
    return true;
  }
  res.success = false;
  res.message = "cannot start if not in 'init' state";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleEndService(
  std_srvs::Trigger::Request & req,
  std_srvs::Trigger::Response & res)
{
  gzdbg << "Handle end service called\n";
  (void)req;
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

  this->dataPtr->currentState = "end_game";
  res.success = true;
  res.message = "competition ended successfully!";
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleSubmitTrayService(
  ros::ServiceEvent<osrf_gear::SubmitTray::Request, osrf_gear::SubmitTray::Response> & event)
{
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  const osrf_gear::SubmitTray::Request& req = event.getRequest();
  osrf_gear::SubmitTray::Response& res = event.getResponse();

  const std::string& callerName = event.getCallerName();
  gzdbg << "Submit tray service called by: " << callerName << std::endl;

  if (this->dataPtr->competitonMode && callerName.compare("/gazebo") != 0)
  {
    std::string errStr = "Competition mode is enabled so this service is not enabled.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    res.success = false;
    return true;
  }

  if (this->dataPtr->currentState != "go") {
    std::string errStr = "Competition is not running so trays cannot be submitted.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
    return false;
  }

  ariac::KitTray kitTray;
  gzdbg << "SubmitTray request received for tray: " << req.tray_id << std::endl;
  if (!this->dataPtr->ariacScorer.GetTrayById(req.tray_id, kitTray))
  {
    res.success = false;
    return true;
  }
  kitTray.currentKit.kitType = req.kit_type;
  res.success = true;
  res.inspection_result = this->dataPtr->ariacScorer.SubmitTray(kitTray).total();
  gzdbg << "Inspection result: " << res.inspection_result << std::endl;
  return true;
}

/////////////////////////////////////////////////
bool ROSAriacTaskManagerPlugin::HandleGetMaterialLocationsService(
  osrf_gear::GetMaterialLocations::Request & req,
  osrf_gear::GetMaterialLocations::Response & res)
{
  gzdbg << "Get material locations service called\n";
  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  auto it = this->dataPtr->materialLocations.find(req.material_type);
  if (it == this->dataPtr->materialLocations.end())
  {
    gzdbg << "No known locations for material type: " << req.material_type << std::endl;
  }
  else
  {
    auto locations = it->second;
    for (auto storage_unit : locations)
    {
      osrf_gear::StorageUnit storageUnitMsg;
      storageUnitMsg.unit_id = storage_unit;
      res.storage_units.push_back(storageUnitMsg);
    }
  }
  return true;
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::ControlConveyorBelt(double power)
{
  gzdbg << "Control conveyor belt called.\n";

  if (!this->dataPtr->conveyorControlClient.exists())
  {
    this->dataPtr->conveyorControlClient.waitForExistence();
  }

  // Make a service call to set the velocity of the belt
  osrf_gear::ConveyorBeltState controlMsg;
  controlMsg.power = power;
  osrf_gear::ConveyorBeltControl srv;
  srv.request.state = controlMsg;
  this->dataPtr->conveyorControlClient.call(srv);
  if (!srv.response.success) {
    std::string errStr = "Failed to control conveyor.";
    gzerr << errStr << std::endl;
    ROS_ERROR_STREAM(errStr);
  }
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::PopulateConveyorBelt()
{
  gzdbg << "Populate conveyor belt called.\n";
  // Publish a message on the activation_plugin of the PopulationPlugin.
  gazebo::msgs::GzString msg;
  msg.set_data("restart");
  this->dataPtr->populatePub->Publish(msg);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::AssignOrder(const ariac::Order & order)
{
    // Publish the order to ROS topic
    std::ostringstream logMessage;
    logMessage << "Announcing order: " << order.orderID << std::endl;
    ROS_ERROR_STREAM(("[INFO] " + logMessage.str()).c_str());
    gzdbg << logMessage.str() << std::endl;

    osrf_gear::Order orderMsg;
    fillOrderMsg(order, orderMsg);
    this->dataPtr->orderPub.publish(orderMsg);

    // Assign the scorer the order to monitor
    gzdbg << "Assigning order: " << order << std::endl;
    this->dataPtr->ariacScorer.AssignOrder(order);
}

/////////////////////////////////////////////////
void ROSAriacTaskManagerPlugin::StopCurrentOrder()
{
  // Stop the current order; any previous orders that are incomplete will automatically be resumed
  if (this->dataPtr->ordersInProgress.size())
  {
    auto orderID = this->dataPtr->ordersInProgress.top().orderID;
    gzdbg << "Stopping order: " << orderID << std::endl;
    this->dataPtr->ordersInProgress.pop();
    this->dataPtr->ariacScorer.UnassignOrder(orderID);
  }
}
