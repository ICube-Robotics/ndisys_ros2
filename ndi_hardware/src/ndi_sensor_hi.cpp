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

#include "helper_capi_functions.h"
/* -------------------------------------------------------------------------- */
/*                         Thing to Make in Parameters                        */
/* -------------------------------------------------------------------------- */

#define __HOSTNAME "192.55.1.80"
#define __SROM "/home/adnan/Desktop/ros2_/ur5_control/src/polaris_vega_interface/sroms/UfoGuideTransducer.rom"

/* -------------------- End: Things to make in parameters ------------------- */

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
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace ndi_hardware
{
    // ------------------------------------------------------------------------------------------
    CallbackReturn NdiSensorHardwareInterface::on_init(
        const hardware_interface::HardwareInfo &info)
    {
        if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::ERROR;

        hw_tracker_poses_.resize(info_.sensors.size(),
                                 std::vector<double>(7, std::numeric_limits<double>::quiet_NaN()));

        /* -------------------------------------------------------------------------- */
        /*                       NDI Connection and Tool Loading                      */
        /* -------------------------------------------------------------------------- */

        RCLCPP_INFO(rclcpp::get_logger("NdiSensorHardwareInterface"), "Starting ...please wait...");

        /* --------------------------- Attempt Connection --------------------------- */
        if (capi.connect(__HOSTNAME) != 0)
        {
            // Print the error and exit if we can't connect to a device
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Connection Failed to host: " << __HOSTNAME);
            return CallbackReturn::FAILURE;
        }
        RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Connection successful ... Firmware version: " << capi.getUserParameter("Features.Firmware.Version"));

        /* ----- Initialize the system. This clears all previously loaded tools ----- */
        onErrorPrintDebugMessage("capi.initialize()", capi.initialize());

        /* -------------------- Load ROM files (tool disc. bins) -------------------- */
        RCLCPP_INFO(rclcpp::get_logger("NdiSensorHardwareInterface"), "Loading ROM files ...");

        {
            std::string error_code = loadTool(__SROM);
            if (!strcmp(error_code.c_str(),"no"))
            {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Loading ROM files ERROR ... " << error_code);
            }
        }

        determineApiSupportForBX2();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "BX2 support: " << apiSupportsBX2);

        /* ----------------------- Initilaize and Enable Tools ---------------------- */
        {
            std::vector<PortHandleInfo> portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
            for (int i = 0; i < portHandles.size(); i++)
            {
                onErrorPrintDebugMessage("capi.portHandleInitialize()", capi.portHandleInitialize(portHandles[i].getPortHandle()));
                onErrorPrintDebugMessage("capi.portHandleEnable()", capi.portHandleEnable(portHandles[i].getPortHandle()));
            }

            // Print all enabled tools
            portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
            for (int i = 0; i < portHandles.size(); i++)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "NDI Hardware Iterface: " << portHandles[i].toString());
            }
        }

        /* -------------------------------------------------------------------------- */
        /*                     End NDI Connection and Tool Loading                    */
        /* -------------------------------------------------------------------------- */

        return CallbackReturn::SUCCESS;
    }
    // ------------------------------------------------------------------------------------------
    std::vector<hardware_interface::StateInterface>
    NdiSensorHardwareInterface::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (uint s = 0; s < info_.sensors.size(); s++)
        {
            for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++)
            {
                state_interfaces.emplace_back(hardware_interface::StateInterface(
                    info_.sensors[s].name, info_.sensors[s].state_interfaces[i].name, &hw_tracker_poses_[s][i]));
            }
        }

        return state_interfaces;
    }
    // ------------------------------------------------------------------------------------------
    CallbackReturn NdiSensorHardwareInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        this->portHandles = capi.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
        if (portHandles.size() < 1)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Cannot publish data when no tools are enabled!");
            return CallbackReturn::ERROR;
        }

        for (int i = 0; i < portHandles.size(); i++)
        {
            enabledTools.push_back(ToolData());
            enabledTools.back().transform.toolHandle = (uint16_t)capi.stringToInt(portHandles[i].getPortHandle());
            enabledTools.back().toolInfo = getToolInfo(portHandles[i].getPortHandle());
        }

        // Start tracking
        RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Entering tracking mode");
        onErrorPrintDebugMessage("Tracking: ", capi.startTracking());

        return CallbackReturn::SUCCESS;
    }
    // ------------------------------------------------------------------------------------------
    CallbackReturn NdiSensorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"), "Deactivating ...");
        return CallbackReturn::SUCCESS;
    }
    // ------------------------------------------------------------------------------------------
    hardware_interface::return_type NdiSensorHardwareInterface::read()
    {
        std::vector<ToolData> newToolData =
            apiSupportsBX2 ? capi.getTrackingDataBX2

                             ("--6d=tools --3d=tools --sensor=none --1d=none")
                           : capi.getTrackingDataBX(TrackingReplyOption::TransformData | TrackingReplyOption::AllTransforms);
        // Update enabledTools array with new data
        for (size_t t = 0; t < enabledTools.size(); t++)
        {
            for (size_t j = 0; j < newToolData.size(); j++)
            {
                if (enabledTools[t].transform.toolHandle == newToolData[j].transform.toolHandle)
                {
                    // Copy the new tool data
                    newToolData[j].toolInfo = enabledTools[t].toolInfo; // keep the serial number
                    enabledTools[t] = newToolData[j];                   // use the new data
                }
            }
        }

        for (size_t t = 0; t < enabledTools.size(); t++)
        {
            
            enabledTools[t].dataIsNew = false;

            std::vector<double> tracker__pose__;
            tracker__pose__.resize(info_.sensors.size(),7);



            tracker__pose__.at(0) = enabledTools[t].transform.q0;
            tracker__pose__.at(1) = enabledTools[t].transform.qx;
            tracker__pose__.at(2) = enabledTools[t].transform.qy;
            tracker__pose__.at(3) = enabledTools[t].transform.qz;

            tracker__pose__.at(4) = enabledTools[t].transform.tx;
            tracker__pose__.at(5) = enabledTools[t].transform.ty;
            tracker__pose__.at(6) = enabledTools[t].transform.tz;



            hw_tracker_poses_.at(t) = tracker__pose__;

            
        }

        return hardware_interface::return_type::OK;
    }
} // namespace ndi_hardware
// ------------------------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ndi_hardware::NdiSensorHardwareInterface, hardware_interface::SensorInterface)