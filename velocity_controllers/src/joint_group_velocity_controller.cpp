// Copyright 2020 PAL Robotics S.L.
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

#include <cmath>
#include <string>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/parameter.hpp"
#include "velocity_controllers/joint_group_velocity_controller.hpp"

namespace velocity_controllers
{
JointGroupVelocityController::JointGroupVelocityController()
: forward_command_controller::ForwardCommandController(),
  latest_command_timestamp_(nullptr)
{
  interface_name_ = hardware_interface::HW_IF_VELOCITY;
}

controller_interface::CallbackReturn JointGroupVelocityController::on_init()
{
  auto ret = ForwardCommandController::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  try
  {
    // Explicitly set the interface parameter declared by the forward_command_controller
    // to match the value set in the JointGroupVelocityController constructor.
    get_node()->set_parameter(
      rclcpp::Parameter("interface_name", hardware_interface::HW_IF_VELOCITY));
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  cmd_timeout_ = get_node()->get_parameter("command_timeout").as_double();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointGroupVelocityController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto ret = this->read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  joints_command_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      rt_command_ptr_.writeFromNonRT(msg);
      latest_command_timestamp_.writeFromNonRT(std::make_shared<rclcpp::Time>(get_node()->get_clock()->now()));
    });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointGroupVelocityController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_activate(previous_state);
  latest_command_timestamp_ = realtime_tools::RealtimeBuffer<std::shared_ptr<rclcpp::Time>>(nullptr);
  return ret;
}

controller_interface::CallbackReturn JointGroupVelocityController::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = ForwardCommandController::on_deactivate(previous_state);
  latest_command_timestamp_ = realtime_tools::RealtimeBuffer<std::shared_ptr<rclcpp::Time>>(nullptr);

  // stop all joints
  for (auto & command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  return ret;
}

controller_interface::return_type JointGroupVelocityController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
    auto joint_commands = rt_command_ptr_.readFromRT();
    auto latest_timestamp = latest_command_timestamp_.readFromRT();

  // no command received yet
  if (!joint_commands || !(*joint_commands) || !latest_timestamp || !(*latest_timestamp))
  {
    return controller_interface::return_type::OK;
  }

  // Check if last command is timed out
  if(cmd_timeout_ > 0 && *(latest_timestamp->get()) + rclcpp::Duration::from_seconds(cmd_timeout_) < time){
      RCLCPP_INFO(get_node()->get_logger(), "No recent commands received, stopping motion");
      // Latest command timed out, stop all joints
      for (auto & command_interface : command_interfaces_)
      {
        command_interface.set_value(0.0);
      }
      return controller_interface::return_type::OK;
  }

  if ((*joint_commands)->data.size() != command_interfaces_.size())
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *(get_node()->get_clock()), 1000,
      "command size (%zu) does not match number of interfaces (%zu)",
      (*joint_commands)->data.size(), command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }

  for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  {
    command_interfaces_[index].set_value((*joint_commands)->data[index]);
  }

  return controller_interface::return_type::OK;
  }

}  // namespace velocity_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  velocity_controllers::JointGroupVelocityController, controller_interface::ControllerInterface)
