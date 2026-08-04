// Copyright (c) 2021 Franka Emika GmbH
// Modifications Copyright (c) 2026 Alberto Bono
//
// Licensed under the Apache License, Version 2.0.

#ifndef ALBERTO_CONTROLLERS_PANDA_JOINT_IMPEDANCE_CONTROLLER_HPP_
#define ALBERTO_CONTROLLERS_PANDA_JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <array>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <chrono>

#include <Eigen/Core>

#include <controller_interface/controller_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace alberto_controllers
{
namespace panda
{

using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;

class JointImpedanceController final
  : public controller_interface::ControllerInterface
{
public:
  static constexpr std::size_t NUM_JOINTS = 7;

  using Vector7d =
    Eigen::Matrix<double, NUM_JOINTS, 1>;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

private:
  void updateJointStates();

  void initializeSubscriber();

  void jointVelocityCommandCallback(
    const std_msgs::msg::Float64MultiArray& msg);

  std::string arm_id_;
  std::string command_topic_;

  double command_timeout_{0.5};
  double velocity_filter_alpha_{0.99};

  Vector7d q_{Vector7d::Zero()};
  Vector7d dq_{Vector7d::Zero()};

  Vector7d q_goal_{Vector7d::Zero()};

  Vector7d dq_desired_{Vector7d::Zero()};
  Vector7d dq_desired_filtered_{Vector7d::Zero()};
  Vector7d dq_filtered_{Vector7d::Zero()};

  Vector7d k_gains_{Vector7d::Zero()};
  Vector7d d_gains_{Vector7d::Zero()};

  std::chrono::steady_clock::time_point last_command_time_{};
  std::mutex command_mutex_;

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_velocity_subscriber_;
};

}  // namespace panda
}  // namespace alberto_controllers

#endif