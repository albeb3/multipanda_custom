// Copyright (c) 2021 Franka Emika GmbH
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

#include "alberto_controllers/panda/dual_joint_velocity_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>




namespace alberto_controllers 
{
namespace panda {

controller_interface::InterfaceConfiguration
DualJointVelocityController::command_interface_configuration() const {
  
  controller_interface::InterfaceConfiguration config;
  
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (std::size_t i = 1; i <= NUM_JOINTS; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
DualJointVelocityController::state_interface_configuration() const {
  
  controller_interface::InterfaceConfiguration config;
  
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_)
  {
    for (std::size_t i = 1; i <= NUM_JOINTS; ++i) 
    {
      const std::string joint_name = arm_container_pair.first + "_joint" + std::to_string(i);
      
      config.names.push_back(joint_name + "/position");
      
      config.names.push_back(joint_name + "/velocity");
    }
  }
  return config;
}

controller_interface::return_type DualJointVelocityController::update( const rclcpp::Time& , const rclcpp::Duration& ) 
{
  if (command_interfaces_.size() != NUM_JOINTS * NUM_ARMS)
  {
    RCLCPP_ERROR_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Expected %zu command interfaces, found %zu", NUM_JOINTS * NUM_ARMS, command_interfaces_.size());
    return controller_interface::return_type::ERROR;
  }
  std::array<double, NUM_JOINTS * NUM_ARMS> command_snapshot{};
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_snapshot = command_;
  }

  rclcpp::Time now = this->get_node()->now();
  if ((now - last_msg_time_).seconds() > 0.5) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "No joint velocity command received for 0.5 seconds. "
      "Setting joint velocities to zero.");
    command_snapshot.fill(0.0); 
  }
  for (std::size_t i = 0; i < NUM_JOINTS * NUM_ARMS; ++i) 
  {
    command_interfaces_[i].set_value(command_snapshot[i]);
  }
  
  return controller_interface::return_type::OK;
}

CallbackReturn DualJointVelocityController::on_init() 
{
  try 
  {
    auto_declare<std::string>("arm_1.arm_id", "mj_left");
    auto_declare<std::string>("arm_2.arm_id", "mj_right");
    auto_declare<std::string>("command_topic", "/joint_velocities");
  }
  catch (const std::exception& e) 
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  command_.fill(0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityController::on_configure( const rclcpp_lifecycle::State& /*previous_state*/) 
{

  for(std::size_t i = 1; i <= NUM_ARMS; i++){
    std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
    rclcpp::Parameter arm_id_param = this->get_node()->get_parameter(arm_id_param_name);
    arms_.insert(std::make_pair(arm_id_param.as_string(), ArmContainer()));
  }

  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
  }
  rclcpp::Parameter command_topic_param = this->get_node()->get_parameter("command_topic");
  command_topic_ = command_topic_param.as_string();
  
  command_.fill(0.0);


  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityController::on_activate(    
  const rclcpp_lifecycle::State& /*previous_state*/) {

  command_.fill(0.0);
 
  last_msg_time_ = this->get_node()->now();

  joint_velocity_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
        command_topic_,1,
        std::bind(
          &DualJointVelocityController::jointVelocityCommandCallback,
          this,std::placeholders::_1));
  
  RCLCPP_INFO(get_node()->get_logger(), "DualJointVelocityController activated. Listening to joint velocity commands on topic: %s", command_topic_.c_str());

  return CallbackReturn::SUCCESS;

}

CallbackReturn DualJointVelocityController::on_deactivate( const rclcpp_lifecycle::State&)
{
  command_.fill(0.0);

  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  joint_velocity_subscriber_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityController::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/){
    command_.fill(0.0);

    RCLCPP_ERROR(this->get_node()->get_logger(), "error encountered!");
    return CallbackReturn::ERROR;
  }

  
void DualJointVelocityController::jointVelocityCommandCallback(const std_msgs::msg::Float64MultiArray& msg) {
  if (msg.data.size() != NUM_JOINTS * NUM_ARMS) 
  {
    RCLCPP_ERROR(get_node()->get_logger(), 
    "Expected %zu joint velocities, received %zu", NUM_JOINTS * NUM_ARMS, msg.data.size());
    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  //   for (size_t i = 0; i < NUM_JOINTS * NUM_ARMS; ++i) {
  //   dq_cmd_[i] = msg.data[i];
  // }
  std::copy(msg.data.begin(), msg.data.end(), command_.begin());

  last_msg_time_ = this->get_node()->now();


}




}  // namespace panda
}  // namespace alberto_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(alberto_controllers::panda::DualJointVelocityController,
                       controller_interface::ControllerInterface)