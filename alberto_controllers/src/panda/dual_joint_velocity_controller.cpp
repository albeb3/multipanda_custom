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


namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace




namespace alberto_controllers {

controller_interface::InterfaceConfiguration
DualJointVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration
DualJointVelocityExampleController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for(auto& arm_container_pair : arms_){
    for (int i = 1; i <= num_joints; ++i) {
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/position");
      config.names.push_back(arm_container_pair.first + "_joint" + std::to_string(i) + "/velocity");
    }
    // config.names.push_back(arm_container_pair.first+"/robot_state");//Define robot state interface to get information (robot state+model)
    // config.names.push_back(arm_container_pair.first+"/robot_model");
  }
  return config;
}

controller_interface::return_type DualJointVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    rclcpp::Time now = this->get_node()->now();
    if ((now - last_msg_time_).seconds() > 0.5) {
      dq_cmd_.fill(0.0); // Set to zero if no command received for 0.5 seconds
    }
    for (size_t i = 0; i < 14; ++i) {
      command_interfaces_[i].set_value(dq_cmd_[i]);
    }
  // updateJointStates();
  // Eigen::Matrix<double, 4, 4> left_arm_pose = getPoseMatrixEigen(left_arm_model);
  // Eigen::Matrix<double, 4, 4> right_arm_pose = getPoseMatrixEigen(right_arm_model);
  // RCLCPP_INFO(get_node()->get_logger(), "Left arm pose: %s", left_arm_pose.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")));
  // RCLCPP_INFO(get_node()->get_logger(), "Right arm pose: %s", right_arm_pose.format(Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]")));

    // for (size_t i= 0; i<7; i++) {
    //   command_interfaces_[i].set_value(dq_left_(i));
    //   command_interfaces_[i+7].set_value(dq_right_(i));
    // }

 
    return controller_interface::return_type::OK;
}

CallbackReturn DualJointVelocityExampleController::on_init() {
  // try {
  //   rclcpp::Parameter arm_count;
  //   bool bHas_arm_count = get_node()->get_parameter("arm_count", arm_count);
  //   //num_robots = get_node()->get_parameter("arm_count").as_int();
  //   if(!bHas_arm_count){
  //     fprintf(stderr, "Failed to get arm_count parameter. Make sure it's set in the yaml file.\n");
  //     return CallbackReturn::ERROR;
  //   }
  //   num_robots = arm_count.as_int();

  // } catch (const std::exception& e) {
  //   fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
  //   return CallbackReturn::ERROR;
  // }
  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {

    for(int i = 1; i <= 2; i++){
    std::string arm_id_param_name = "arm_" + std::to_string(i) + ".arm_id";
    rclcpp::Parameter arm_id_param = this->get_node()->get_parameter(arm_id_param_name);
    arms_.insert(std::make_pair(arm_id_param.as_string(), ArmContainer()));
  }

   int i = 1;
  for(auto& arm_container_pair : arms_){
    auto &arm = arm_container_pair.second;
    arm.arm_id_ = arm_container_pair.first;
    i++;
  }
  dq_left_.setZero();
  dq_right_.setZero();
  dq_cmd_.fill(0.0);


  return CallbackReturn::SUCCESS;
}

CallbackReturn DualJointVelocityExampleController::on_activate(    
  const rclcpp_lifecycle::State& /*previous_state*/) {
  // left_arm_model->assign_loaned_state_interfaces(state_interfaces_); //asign object to variable left arm model
  // right_arm_model->assign_loaned_state_interfaces(state_interfaces_);  
  // updateJointStates();
  start_time_ = this->get_node()->now();
  last_msg_time_ = this->get_node()->now();
  init_time_ = rclcpp::Duration(0, 0);
  DualJointVelocityExampleController::initSubscribers();
  return CallbackReturn::SUCCESS;

}

CallbackReturn DualJointVelocityExampleController::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/){
    RCLCPP_ERROR(this->get_node()->get_logger(), "error encountered!");
    return CallbackReturn::ERROR;
  }

  void DualJointVelocityExampleController::initSubscribers() {

    joint_velocity_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
          "/joint_velocities", 1,
          std::bind(&DualJointVelocityExampleController::jointVelocityCommandCallback, this, std::placeholders::_1)
        );
    
  }
   void DualJointVelocityExampleController::jointVelocityCommandCallback(const std_msgs::msg::Float64MultiArray& msg) {
    if (msg.data.size() != 14) {
      RCLCPP_ERROR(get_node()->get_logger(), "Received joint velocity command with incorrect size: %zu", msg.data.size());
      return;
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
     for (size_t i = 0; i < 14; ++i) {
      dq_cmd_[i] = msg.data[i];
    }
    last_msg_time_ = this->get_node()->now();
    // for (size_t i = 0; i < 14; ++i) {
    //   command_interfaces_[i].set_value(msg.data[i]);
    //   // dq_left_(i) = msg.data[i];
    //   // dq_right_(i) = msg.data[i+7];
    // }

      // command_interfaces_[i].set_value(msg.data[i]);
      // command_interfaces_[i+7].set_value(msg.data[i+7]);
      
 

  


  }




}  // namespace alberto_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(alberto_controllers::DualJointVelocityExampleController,
                       controller_interface::ControllerInterface)