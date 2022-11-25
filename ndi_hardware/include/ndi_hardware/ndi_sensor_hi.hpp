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

#ifndef NDI_HARDWARE__NDI_EFFORT_HI
#define NDI_HARDWARE__NDI_EFFORT_HI

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "ndi_hardware/visibility_control.h"
#include <rclcpp/rclcpp.hpp>

#include "CombinedApi.h"
#include "PortHandleInfo.h"
#include "ToolData.h"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ndi_hardware
{
class NdiSensorHardwareInterface : public hardware_interface::SensorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(NdiSensorHardwareInterface);

  NDI_HARDWARE_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  NDI_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  NDI_HARDWARE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  NDI_HARDWARE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  NDI_HARDWARE_PUBLIC
  hardware_interface::return_type read(
      const rclcpp::Time & time,
      const rclcpp::Duration & period) override;

private:
  // Store the poses of the trackers 
  std::vector<std::vector<double>> hw_tracker_poses_;


  /** @brief Vector holding tool names (file paths) in order of YAML file*/
  std::vector<std::string> tool_names_;
  /** @brief Vector holding currently tracked tools*/
  std::vector<ToolData> enabledTools_;
  /** @brief Vector holding currently tracked tools*/
  std::vector<ToolData> newToolData_;
  /** @brief Vector holding port handles for the trackers (see NDI Api documentation)*/
  std::vector<PortHandleInfo> portHandles_;
  /** @brief Number of tools in the YAML file (tracked and not-tracked)*/
  size_t tool_count_;
  /** @brief IP of the NDI device (string format in the YAML)*/
  std::string ndi_ip_;
  /** @brief True if the connected device supports BX2 request structure*/
  bool apiSupportsBX2_ = false;
  /** @brief Class of functions mainly without real use but organization. 
   * To be stripped and removed*/
  CombinedApi capi_;

  /**
   * @brief Prints a debug message if a method call failed.
   * @details To use, pass the method name and the error code returned by the method.
   *          Eg: onErrorPrintDebugMessage("capi.initialize()", capi.initialize());
   *          If the call succeeds, this method does nothing.
   *          If the call fails, this method prints an error message to stdout.
   */
  void onErrorPrintDebugMessage(std::string methodName, int errorCode);

  /**
   * @brief Determines whether an NDI device supports the BX2 command by looking at the API revision
   * @details To use, assign it to a bool variable and use the variable.
   * The function is expensive, to be called only once in initialization phase only.
   */
  bool determineApiSupportForBX2();

  /**
   * @brief Loads a tool from a tool definition file (.rom)
   * @details To use, pass a const char* to the function containing the file path of the ROM bins.
   */
  void loadTool(const char *toolDefinitionFilePath);

  /**
   * @brief Initialize and enable loaded tools. This is the same regardless of tool type.
   */
  void initializeAndEnableTools();

  /**
   * @brief Returns the string: "[tool.id] s/n:[tool.serialNumber]" used in CSV output
   */
  std::string getToolInfo(std::string toolHandle);
  /**
   * @brief Returns a rclcpp::Parameter object with params inside.
   */
  void getParamsFromFile(std::string config_file);

};

}  // namespace NDI_HARDWARE

#endif  // NDI_HARDWARE__NDI_EFFORT_HI