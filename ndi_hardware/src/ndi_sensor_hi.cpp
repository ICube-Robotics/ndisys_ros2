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
// Authors: Maciej Bednarczyk, Adnan SAOOD
//

#include "ndi_hardware/ndi_sensor_hi.hpp"

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

#include "ndi_capi/include/CombinedApi.h"
#include "ndi_capi/include/PortHandleInfo.h"
#include "ndi_capi/include/ToolData.h"

namespace ndi_hardware
{
// ------------------------------------------------------------------------------------------
CallbackReturn NdiSensorHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS){
        return CallbackReturn::ERROR;
    }

    capi_ = CombinedApi();

    /* Get params ........................................................... */
    getParamsFromDesc();

    /* Attemp Connect ....................................................... */
    if (capi_.connect(ndi_ip_) != 0){
        // Print the error and exit if we can't connect to a device
        std::cout << "Connection Failed to host: " << ndi_ip_ << std::endl;
        return CallbackReturn::ERROR;
    }

    /* Wait for 1 sec then display connection ............................... */
    rclcpp::sleep_for(std::chrono::seconds(1));
    std::cout << "Connected to host: "
                << ndi_ip_
                << " With firmware version: "
                << capi_.getUserParameter("Features.Firmware.Version")
                << std::endl;

    /* Determine Support for BX2 ........................................ */
    determineApiSupportForBX2();

    /* Init. the system and clear tools ................................. */
    onErrorPrintDebugMessage("capi_.initialize()", capi_.initialize());

    /* Load ROM bins .................................................... */
    for (size_t i = 0; i < tool_count_; i++){
        loadTool(tool_names_.at(i).c_str());
    }

    initializeAndEnableTools();
    portHandles_ = capi_.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);

    if (portHandles_.size() < 1){
        std::cout << "Cannot publish data file when no tools are enabled!" << std::endl;
        return CallbackReturn::FAILURE;
    }

    for (int i = 0; i < portHandles_.size(); i++){
        enabledTools_.push_back(ToolData());
        enabledTools_.back().transform.toolHandle = (uint16_t)capi_.stringToInt(portHandles_[i].getPortHandle());
        enabledTools_.back().toolInfo = getToolInfo(portHandles_[i].getPortHandle());
    }

    hw_tracker_poses_.resize(info_.sensors.size(), 
        std::vector<double>(7, std::numeric_limits<double>::quiet_NaN()));


    return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
NdiSensorHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (uint s = 0; s < info_.sensors.size(); s++){ // num
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
   
    int tracking_return_code = capi_.startTracking();
    onErrorPrintDebugMessage("Tracking: ", tracking_return_code);
    return (tracking_return_code < 0 ? CallbackReturn::FAILURE : CallbackReturn::SUCCESS);  
}
// ------------------------------------------------------------------------------------------
CallbackReturn NdiSensorHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    return CallbackReturn::SUCCESS;
}
// ------------------------------------------------------------------------------------------
hardware_interface::return_type NdiSensorHardwareInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    
    newToolData_ = apiSupportsBX2_
                        ? capi_.getTrackingDataBX2("--6d=tools --3d=tools --sensor=none --1d=none")
                        : capi_.getTrackingDataBX(TrackingReplyOption::TransformData |
                                                 TrackingReplyOption::AllTransforms);

    // Update enabledTools array with new data
    for (size_t t = 0; t < enabledTools_.size(); t++){
        for (size_t j = 0; j < newToolData_.size(); j++){
            if (enabledTools_[t].transform.toolHandle == newToolData_[j].transform.toolHandle){
                // Copy the new tool data
                newToolData_[j].toolInfo = enabledTools_[t].toolInfo; // keep the serial number
                enabledTools_[t] = newToolData_[j];                   // use the new data
            }
        }
    }

    // Store one pose of the trackers
    std::vector<double> tracker_pose_(7);
    
    for (size_t t = 0; t < enabledTools_.size(); t++){
        
        enabledTools_[t].dataIsNew = false;

        tracker_pose_.at(0) = enabledTools_[t].transform.q0;
        tracker_pose_.at(1) = enabledTools_[t].transform.qx;
        tracker_pose_.at(2) = enabledTools_[t].transform.qy;
        tracker_pose_.at(3) = enabledTools_[t].transform.qz;
        tracker_pose_.at(4) = enabledTools_[t].transform.tx / 1000.0;
        tracker_pose_.at(5) = enabledTools_[t].transform.ty / 1000.0;
        tracker_pose_.at(6) = enabledTools_[t].transform.tz / 1000.0;

        hw_tracker_poses_.at(t) = tracker_pose_;
    }
    
    return hardware_interface::return_type::OK;
}
// ------------------------------------------------------------------------------------------

/* -------------------------------------------------------------------------- */
/*                                NDI Functions                               */
/* -------------------------------------------------------------------------- */
void NdiSensorHardwareInterface::onErrorPrintDebugMessage(std::string methodName, int errorCode)
{
    if (errorCode < 0){
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("onErrorPrintDebugMessage"),
                        "NDI Hardware: " << methodName << " failed: " << capi_.errorToString(errorCode));
    }
}
// ------------------------------------------------------------------------------------------
bool NdiSensorHardwareInterface::determineApiSupportForBX2()
{
    // Lookup the API revision
    std::string response = capi_.getApiRevision();

    // Refer to the API guide for how to interpret the APIREV response
    char deviceFamily = response[0];
    int majorVersion = capi_.stringToInt(response.substr(2, 3));

    // As of early 2017, the only NDI device supporting BX2 is the Vega
    // Vega is a Polaris device with API major version 003
    if (deviceFamily == 'G' && majorVersion >= 3){
        return true;
    }
    return false;
}
// ------------------------------------------------------------------------------------------
void NdiSensorHardwareInterface::loadTool(const char *toolDefinitionFilePath)
{
    // Request a port handle to load a passive tool into
    int portHandle = capi_.portHandleRequest();
    onErrorPrintDebugMessage("capi_.portHandleRequest()", portHandle);
    // Load the .rom file using the previously obtained port handle
    capi_.loadSromToPort(toolDefinitionFilePath, portHandle);
}
// ------------------------------------------------------------------------------------------
void NdiSensorHardwareInterface::initializeAndEnableTools()
{
    // Initialize and enable tools
    std::vector<PortHandleInfo> portHandles = 
            capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (int i = 0; i < portHandles.size(); i++)
    {
        onErrorPrintDebugMessage("capi_.portHandleInitialize()",
                                    capi_.portHandleInitialize(portHandles[i].getPortHandle()));
        onErrorPrintDebugMessage("capi_.portHandleEnable()",
                                    capi_.portHandleEnable(portHandles[i].getPortHandle()));
    }
}
// ------------------------------------------------------------------------------------------
std::string NdiSensorHardwareInterface::getToolInfo(std::string toolHandle){
    // Get the port handle info from PHINF
    PortHandleInfo info = capi_.portHandleInfo(toolHandle);
    
    // Return the ID and SerialNumber the desired string format
    std::string outputString = info.getToolId();
    outputString.append(" s/n:").append(info.getSerialNumber());
    return outputString;
}
// ------------------------------------------------------------------------------------------
void NdiSensorHardwareInterface::getParamsFromDesc()
{
    ndi_ip_ = info_.hardware_parameters["ndi_ip"];  //params["polaris_ip"].as<std::string>();
    tool_count_ = info_.sensors.size();
    std::cout << "Loading " << tool_count_ << " tools" << std::endl;
    for (size_t i = 0; i < tool_count_; i++)
    {
        tool_names_.push_back(info_.sensors.at(i).parameters["srom"]);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("NdiSensorHardwareInterface"),
                        "Reading tracker [" << info_.sensors.at(i).name << "] in "
                        << info_.sensors.at(i).parameters["srom"]);
    }
    return;
}

/* -------------------------------------------------------------------------- */
/*                             End: NDI Functions                             */
/* -------------------------------------------------------------------------- */

}  // namespace ndi_hardware
// ------------------------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ndi_hardware::NdiSensorHardwareInterface, hardware_interface::SensorInterface)