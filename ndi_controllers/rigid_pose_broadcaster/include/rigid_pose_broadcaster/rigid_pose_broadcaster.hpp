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

#ifndef RIGID_POSE_BROADCASTER__RIGID_POSE_BROADCASTER_HPP_
#define RIGID_POSE_BROADCASTER__RIGID_POSE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>


#include "controller_interface/controller_interface.hpp"
#include "rigid_pose_broadcaster/visibility_control.h"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "ndi_msgs/msg/rigid_array.hpp"

namespace rigid_pose_broadcaster
{
class RigidPoseBroadcaster : public controller_interface::ControllerInterface
{
public:
  RIGID_POSE_BROADCASTER_PUBLIC
  RigidPoseBroadcaster();

  RIGID_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  RIGID_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  RIGID_POSE_BROADCASTER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  RIGID_POSE_BROADCASTER_PUBLIC
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  RIGID_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  RIGID_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  RIGID_POSE_BROADCASTER_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  //  For the PoseStamped message,
  std::shared_ptr<rclcpp::Publisher<ndi_msgs::msg::RigidArray>> rigid_pose_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<ndi_msgs::msg::RigidArray>> realtime_rigid_pose_publisher_;
  std::unordered_map<std::string, double> name_if_value_mapping_;

};

}  // namespace rigid_pose_broadcaster

#endif  // RIGID_POSE_BROADCASTER__RIGID_POSE_BROADCASTER_HPP_