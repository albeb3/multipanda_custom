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

#include <franka_example_controllers/comless/multi_joint_impedance_example_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
MultiJointImpedanceExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/effort");
    }
  }

  return config;
}

controller_interface::InterfaceConfiguration
MultiJointImpedanceExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::return_type MultiJointImpedanceExampleController::update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
  updateJointStates();
  size_t k = 0;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    if (arm.first_update_) {
      arm.q_goal_ = arm.q_;
      arm.first_update_ = false;
    }
    Vector7d dq_des_local;
    {
      std::lock_guard<std::mutex> lock(command_mutex_);
      dq_des_local = arm.dq_des_;
    }
    rclcpp::Time now = this->get_node()->now();
    if ((now - last_msg_time_).seconds() > 0.5) {
      RCLCPP_WARN(get_node()->get_logger(), "No joint velocity command received for more than 0.5 seconds.");
      dq_des_local.setZero(); // Set to zero if no command received for 0.5 seconds
    }
    double dt = period.seconds();
    //arm.q_goal_ += dq_des_local * dt;
    arm.q_goal_ = arm.q_ + dq_des_local * dt; 

    const double kAlpha = 0.99;
    arm.dq_desired_filtered_ = dq_des_local;
    arm.dq_filtered_ = (1 - kAlpha) * arm.dq_filtered_ + kAlpha * arm.dq_;
    Vector7d tau_d_calculated =
        arm.k_gains_.cwiseProduct(arm.q_goal_ - arm.q_) + arm.d_gains_.cwiseProduct(arm.dq_desired_filtered_ - arm.dq_filtered_);

    for (int i = 0; i < num_joints; i++) {
      command_interfaces_[k].set_value(tau_d_calculated(i));
      k++; // BIG assumption: That the command interfaces are always in the same order
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn MultiJointImpedanceExampleController::on_init() {
  try {
    rclcpp::Parameter arm_count;
    bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
    //num_robots = get_node()->get_parameter("arm_count").as_int();
    if(!bHas_arm_count){
      fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n");
      return CallbackReturn::ERROR;
    }
    num_robots = arm_count.as_int();

  } catch (const std::exception& e) {
    fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n%s \n", e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Finished initializing multi joint impedance example controller for %d arms", num_robots);
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiJointImpedanceExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  
  for(int i = 1; i <= num_robots; i++){
    std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
    rclcpp::Parameter arm_id_param = this->get_node()->get_parameter(arm_id_param_name);
    arms_.insert(std::make_pair(arm_id_param.as_string(), ArmContainer()));
  }
  
  int i = 1;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
    auto k_gains = get_node()->get_parameter("arm_" + std::to_string(i) + ".k_gains").as_double_array();
    auto d_gains = get_node()->get_parameter("arm_" + std::to_string(i) + ".d_gains").as_double_array();

    if (k_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (k_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                  num_joints, k_gains.size());
      return CallbackReturn::FAILURE;
    }
    if (d_gains.empty()) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains parameter not set");
      return CallbackReturn::FAILURE;
    }
    if (d_gains.size() != static_cast<uint>(num_joints)) {
      RCLCPP_FATAL(get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                  num_joints, d_gains.size());
      return CallbackReturn::FAILURE;
    }
    for (int i = 0; i < num_joints; ++i) {
      arm.d_gains_(i) = d_gains.at(i);
      arm.k_gains_(i) = k_gains.at(i);
    }
    arm.dq_filtered_.setZero();
    i++;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Finished configuring multi joint impedance example controller");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MultiJointImpedanceExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  updateJointStates();
  MultiJointImpedanceExampleController::initSubscribers();
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.initial_q_ = arm.q_;
    arm.q_goal_ = arm.q_;
    arm.first_update_ = true;
    arm.dq_des_.setZero();
    arm.dq_desired_filtered_.setZero();
    arm.dq_filtered_.setZero();
  }

  // LOG INTERFACCE
  RCLCPP_INFO(get_node()->get_logger(), "=== COMMAND INTERFACES (%zu) ===", command_interfaces_.size());
  for (auto& ci : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  CMD: '%s' / '%s'",
      ci.get_prefix_name().c_str(), ci.get_interface_name().c_str());
  }
  RCLCPP_INFO(get_node()->get_logger(), "=== STATE INTERFACES (%zu) ===", state_interfaces_.size());
  for (auto& si : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "  STATE: '%s' / '%s'",
      si.get_prefix_name().c_str(), si.get_interface_name().c_str());
  }

  start_time_ = this->get_node()->now();
  last_msg_time_ = this->get_node()->now();
  return CallbackReturn::SUCCESS;
}

void MultiJointImpedanceExampleController::updateJointStates() {
  
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    size_t k = 0;
    for (size_t i = 0; i < state_interfaces_.size(); i++) {
      const auto& position_interface = state_interfaces_.at(2 * i);
      const auto& velocity_interface = state_interfaces_.at(2 * i + 1);
      if(position_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos || 
         velocity_interface.get_prefix_name().find(arm_container_pair.first) == std::string::npos ){
          // if either position or velocity interface does not contain the ID of the arm, skip
          continue;
      };

      assert(position_interface.get_interface_name() == "position");
      assert(velocity_interface.get_interface_name() == "velocity");

      arm.q_(k) = position_interface.get_value();
      arm.dq_(k) = velocity_interface.get_value();

      k++;
      if(k == 7){
        break;
      }
    }
  }
}
void MultiJointImpedanceExampleController::initSubscribers() {

    joint_velocity_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/joint_velocities", 1,
          std::bind(&MultiJointImpedanceExampleController::jointVelocityCommandCallback, this, std::placeholders::_1)
        );
    
  }
void MultiJointImpedanceExampleController::jointVelocityCommandCallback(
    const std_msgs::msg::Float64MultiArray& msg) {
  std::lock_guard<std::mutex> lock(command_mutex_);
  
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
    "Received velocity cmd: left[0]=%.4f right[0]=%.4f",
    msg.data.size() > 0 ? msg.data[0] : 0.0,
    msg.data.size() > 7 ? msg.data[7] : 0.0);

  if (msg.data.size() != 14) {
    RCLCPP_ERROR(get_node()->get_logger(), "Wrong size: %zu", msg.data.size());
    return;
  }

  for (auto& arm_container_pair : arms_) {
    const std::string& arm_id = arm_container_pair.first;
    auto& arm = arm_container_pair.second;

    if (arm_id.find("left") != std::string::npos) {
      for (int i = 0; i < 7; ++i)
        arm.dq_des_(i) = msg.data[i];
    } else if (arm_id.find("right") != std::string::npos) {
      for (int i = 0; i < 7; ++i)
        arm.dq_des_(i) = msg.data[i + 7];
    } else {
      RCLCPP_WARN_ONCE(get_node()->get_logger(),
        "arm_id '%s' non contiene 'left' o 'right', ignorato", arm_id.c_str());
    }
  }
  last_msg_time_ = this->get_node()->now();
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::MultiJointImpedanceExampleController,
                       controller_interface::ControllerInterface)