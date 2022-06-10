// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Authors: Maciej Bednarczyk
//

#include "ndi_hardware/ndi_sensor_hi.hpp"

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <sstream>
#include <iterator>
#include <iostream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ndi_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn NdiSensorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

    hw_tracker_poses_.resize(info_.sensors.size(), 
        std::vector<double>(7, std::numeric_limits<double>::quiet_NaN()));

    return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
NdiSensorHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint s = 0; s < info_.sensors.size(); s++){
        for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++){
            state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[s].name, info_.sensors[s].state_interfaces[i].name, &hw_tracker_poses_[s][i]));
        }
    }
    
    return state_interfaces;
}
// ------------------------------------------------------------------------------------------
CallbackReturn NdiSensorHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(rclcpp::get_logger("NdiSensorHardwareInterface"), "Starting ...please wait...");
    return CallbackReturn::SUCCESS;
  
}
// ------------------------------------------------------------------------------------------
CallbackReturn NdiSensorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type NdiSensorHardwareInterface::read()
{
    return hardware_interface::return_type::OK;
}
}  // namespace ndi_hardware
// ------------------------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ndi_hardware::NdiSensorHardwareInterface, hardware_interface::SensorInterface)