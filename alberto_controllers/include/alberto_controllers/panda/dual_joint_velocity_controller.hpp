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

#pragma once

#include <string>
#include <rclcpp/time_source.hpp>
#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
/*
for test
*/

#include <geometry_msgs/msg/transform_stamped.hpp>



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace alberto_controllers {
namespace panda {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class DualJointVelocityController : public controller_interface::ControllerInterface {
 public:
  static constexpr std::size_t NUM_ARMS = 2;
  static constexpr std::size_t NUM_JOINTS = 7;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector14d = Eigen::Matrix<double, 14, 1>;
  struct ArmContainer{
    std::string arm_id_;
    Vector7d q_;
    Vector7d dq_;
  };
  std::map<std::string, ArmContainer> arms_;
  
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;
 private:
  void jointVelocityCommandCallback(const std_msgs::msg::Float64MultiArray& msg);

  std::string arm_id_;
  std::string command_topic_;

  std::array<double, NUM_JOINTS * 2> command_{};
  std::mutex command_mutex_;
  rclcpp::Time last_msg_time_;

 
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_velocity_subscriber_;



};
} // namespace panda
}  // namespace alberto_controllers