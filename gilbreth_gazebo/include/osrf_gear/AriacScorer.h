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
 * Desc: ARIAC scorer.
 * Author: Deanna Hood
 */
#ifndef _ROS_ARIAC_SCORER_HH_
#define _ROS_ARIAC_SCORER_HH_

#include <map>
#include <string>

#include <ros/ros.h>

#include "osrf_gear/ARIAC.hh"
#include "osrf_gear/AriacKitTray.h"
#include <osrf_gear/KitTray.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/TrayContents.h>
#include "osrf_gear/VacuumGripperState.h"

/// \brief A scorer for the ARIAC game.
class AriacScorer
{
  /// \brief Constructor.
  public: AriacScorer();

  /// \brief Destructor.
  public: virtual ~AriacScorer();

  /// \brief Update the scorer.
  public: void Update(double timeStep = 0.0);

  /// \brief Get the current score.
  /// \param[in] The time in seconds since the last update.
  /// \return The score for the game.
  public: ariac::GameScore GetGameScore();

  /// \brief Get the score of the current order.
  /// \return True if the order is complete.
  public: bool IsOrderComplete(const ariac::OrderID_t & orderID);

  /// \brief Get the score of the current order.
  /// \return The score for the order.
  public: ariac::OrderScore GetOrderScore(const ariac::OrderID_t & orderID);

  /// \brief Assign an order to process.
  /// \param[in] order The order.
  public: void AssignOrder(const ariac::Order & order);

  /// \brief Stop processing the current order.
  /// \param[in] timeTaken The time spent on the order.
  /// \return The score for the order.
  public: ariac::OrderScore UnassignOrder(const ariac::OrderID_t & orderID);

  /// \brief Get the kit trays the scorer is monitoring.
  /// \return Vector of kit tray states.
  public: std::vector<ariac::KitTray> GetTrays();

  /// \brief Get the kit tray with the specified ID.
  /// \param[in] trayID The ID of the tray to get.
  /// \param[in] kitTray The kitTray found.
  /// \return True if the tray was found, false otherwise.
  public: bool GetTrayById(const ariac::TrayID_t & trayID, ariac::KitTray & kitTray);

  /// \brief Submit tray for scoring and store the result in the order score.
  public: ariac::TrayScore SubmitTray(const ariac::KitTray & tray);

  /// \brief Calculate the score for a tray given the type of kit being built.
  protected: ariac::TrayScore ScoreTray(const ariac::KitTray & tray, const ariac::Kit & assignedKit);

  /// \brief Helper function for filling a Kit from a tray contents ROS message.
  public: static void FillKitFromMsg(const osrf_gear::TrayContents::ConstPtr & trayMsg, ariac::Kit & kit);

  /// \brief Helper function for filling a Kit from a kit ROS message.
  public: static void FillKitFromMsg(const osrf_gear::Kit & kitMsg, ariac::Kit & kit);

  /// \brief Callback for receiving order message.
  public: void OnOrderReceived(const osrf_gear::Order::ConstPtr & orderMsg);

  /// \brief Callback for receiving tray state message.
  public: void OnTrayInfoReceived(const osrf_gear::TrayContents::ConstPtr & trayMsg);

  /// \brief Callback for receiving gripper state message.
  public: void OnGripperStateReceived(const osrf_gear::VacuumGripperState &stateMsg);

  /// \brief The trays to monitor the score of.
  protected: std::map<ariac::TrayID_t, ariac::KitTray> kitTrays;

  /// \brief Mutex for protecting the orders being scored.
  protected: mutable boost::mutex mutex;

  /// \brief Collection of orders that have been announced but are not yet complete.
  protected: std::vector<ariac::Order> ordersInProgress;

  /// \brief Flag for signalling new tray info to process.
  protected: bool newTrayInfoReceived = false;

  /// \brief Flag for signalling new order to process.
  protected: bool newOrderReceived = false;

  /// \brief Whether or not there is a travelling part in the gripper.
  protected: bool isPartTravelling = false;

  /// \brief Order receivd from order messages.
  protected: ariac::Order newOrder;

  /// \brief Parameters to use for calculating scores.
  protected: ariac::ScoringParameters scoringParameters;

  /// \brief Pointer to the score of the current order.
  protected: ariac::OrderScore* orderScore;

  /// \brief The score of the run.
  protected: ariac::GameScore gameScore;

};
#endif
