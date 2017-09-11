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

#include <string>

#include <osrf_gear/ARIAC.hh>

namespace ariac
{
  /// \brief Class to store information about a kit tray.
  class KitTray
  {
    /// \brief Constructor.
    public: KitTray();

    /// \brief Constructor.
    /// \param[in] _trayID ID of the tray.
    /// \param[in] _assignedKit Kit assigned to the tray.
    public: KitTray(std::string _trayID);

    /// \brief Destructor.
    public: ~KitTray();

    /// \brief Update the current state of the kit on the tray.
    public: void UpdateKitState(const Kit & kit);

    /// \brief The ID of the tray.
    public: std::string trayID;

    /// \brief The current state of the kit on the tray.
    public: Kit currentKit;

    /// \brief Flag for signalling the state of the tray has changed.
    protected: bool kitStateChanged;
  };
}
