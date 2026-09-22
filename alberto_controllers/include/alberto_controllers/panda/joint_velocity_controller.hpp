#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <mutex>

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

class JointVelocityController final
  : public controller_interface::ControllerInterface
{
public:
  static constexpr std::size_t NUM_JOINTS = 7;
  

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_init() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

private:
  void jointVelocityCommandCallback(const std_msgs::msg::Float64MultiArray& msg);

  std::string arm_id_;
  std::string command_topic_;

  std::array<double, NUM_JOINTS> command_{};
  std::mutex command_mutex_;
  rclcpp::Time last_msg_time_;
 

  rclcpp::Subscription<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    joint_velocity_subscriber_;
};

}  // namespace panda
}  // namespace alberto_controllers