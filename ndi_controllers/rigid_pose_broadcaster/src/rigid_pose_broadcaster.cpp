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

#include "rigid_pose_broadcaster/rigid_pose_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>


#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/pose.hpp"


namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace rigid_pose_broadcaster
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();

RigidPoseBroadcaster::RigidPoseBroadcaster() {}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn RigidPoseBroadcaster::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RigidPoseBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration RigidPoseBroadcaster::state_interface_configuration()
  const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::ALL};
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RigidPoseBroadcaster::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try{
    rigid_pose_publisher_ = get_node()->create_publisher<ndi_msgs::msg::RigidArray>("rigid_poses", rclcpp::SystemDefaultsQoS());
    realtime_rigid_pose_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ndi_msgs::msg::RigidArray>>(rigid_pose_publisher_);

    this->sensor_names = this->get_node()->get_parameter("sensor_names").as_string_array();
    this->sensor_ids = this->get_node()->get_parameter("sensor_ids").as_integer_array();
  
  }
  catch (const std::exception & e){
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RigidPoseBroadcaster::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RigidPoseBroadcaster::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

double get_value(
  const std::unordered_map<std::string, double> & map,
  const std::string & name)
{
  auto it = map.find(name);
  if (it != map.end())
  {
    return map.at(name);
  }
  else
  {
    return kUninitializedValue;
  }
}

controller_interface::return_type RigidPoseBroadcaster::update(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  for (const auto & state_interface : state_interfaces_)
  {
    name_if_value_mapping_[state_interface.get_name()] = state_interface.get_value();
    RCLCPP_DEBUG(get_node()->get_logger(), "%s: %f\n", state_interface.get_name().c_str(), state_interface.get_value());
  }

  if (realtime_rigid_pose_publisher_ && realtime_rigid_pose_publisher_->trylock())
  {
    realtime_rigid_pose_publisher_->msg_.poses.clear();
    realtime_rigid_pose_publisher_->msg_.ids.clear();

    auto & rigid_pose_msg = realtime_rigid_pose_publisher_->msg_;
    rigid_pose_msg.header.stamp = get_node()->get_clock()->now();
    // TODO: This should come from a paramter
    rigid_pose_msg.header.frame_id = "polaris_base";

    for(size_t iter_ = 0 ; iter_ < sensor_names.size(); iter_++)
    {
      auto sensor_name = this->sensor_names.at(iter_);
      auto sensor_id = this->sensor_ids.at(iter_);
      
      auto p = geometry_msgs::msg::Pose();
      
      p.position.x = get_value(name_if_value_mapping_, sensor_name+"/pose.position.x");
      p.position.y = get_value(name_if_value_mapping_, sensor_name+"/pose.position.y");
      p.position.z = get_value(name_if_value_mapping_, sensor_name+"/pose.position.z");
      p.orientation.w = get_value(name_if_value_mapping_, sensor_name+"/pose.orientation.w");
      p.orientation.x = get_value(name_if_value_mapping_, sensor_name+"/pose.orientation.x");
      p.orientation.y = get_value(name_if_value_mapping_, sensor_name+"/pose.orientation.y");
      p.orientation.z = get_value(name_if_value_mapping_, sensor_name+"/pose.orientation.z");

      rigid_pose_msg.poses.push_back(p);
      rigid_pose_msg.ids.push_back(sensor_id);
    }
    
    realtime_rigid_pose_publisher_->unlockAndPublish();    
  }

  return controller_interface::return_type::OK;
}

}  // namespace rigid_pose_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rigid_pose_broadcaster::RigidPoseBroadcaster, controller_interface::ControllerInterface)