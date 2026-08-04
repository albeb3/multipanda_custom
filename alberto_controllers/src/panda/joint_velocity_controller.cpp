#include "alberto_controllers/panda/joint_velocity_controller.hpp"

#include <algorithm>
#include <exception>
#include <functional>
#include <string>



namespace alberto_controllers
{
namespace panda
{

controller_interface::InterfaceConfiguration
JointVelocityController::
command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type =
    controller_interface::
      interface_configuration_type::INDIVIDUAL;

  for (
    std::size_t i = 1;
    i <= NUM_JOINTS;
    ++i)
  {
    config.names.push_back(
      arm_id_ +
      "_joint" +
      std::to_string(i) +
      "/velocity");
  }

  return config;
}

controller_interface::InterfaceConfiguration
JointVelocityController::
state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type =
    controller_interface::
      interface_configuration_type::INDIVIDUAL;

  for (
    std::size_t i = 1;
    i <= NUM_JOINTS;
    ++i)
  {
    const std::string joint_name =
      arm_id_ +
      "_joint" +
      std::to_string(i);

    config.names.push_back(
      joint_name + "/position");

    config.names.push_back(
      joint_name + "/velocity");
  }

  return config;
}

controller_interface::return_type
JointVelocityController::update(
  const rclcpp::Time&,
  const rclcpp::Duration&)
{
  if (command_interfaces_.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Expected %zu command interfaces, found %zu",
      NUM_JOINTS,
      command_interfaces_.size());

    return controller_interface::return_type::ERROR;
  }
  std::array<double, NUM_JOINTS> command_snapshot{};
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    command_snapshot = command_;
  }

  for (
    std::size_t i = 0;
    i < NUM_JOINTS;
    ++i)
  {
    command_interfaces_[i].set_value(
      command_snapshot[i]);
  }

  return controller_interface::return_type::OK;
}

CallbackReturn
JointVelocityController::on_init()
{
  try
  {
    auto_declare<std::string>(
      "arm_id",
      "panda");

    auto_declare<std::string>(
      "command_topic",
      "/joint_velocities");
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Initialization failed: %s",
      exception.what());

    return CallbackReturn::ERROR;
  }

  command_.fill(0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointVelocityController::on_configure(
  const rclcpp_lifecycle::State&)
{
  arm_id_ =
    get_node()
      ->get_parameter("arm_id")
      .as_string();

  command_topic_ =
    get_node()
      ->get_parameter("command_topic")
      .as_string();

  if (arm_id_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'arm_id' cannot be empty");

    return CallbackReturn::ERROR;
  }

  if (command_topic_.empty())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Parameter 'command_topic' cannot be empty");

    return CallbackReturn::ERROR;
  }

  command_.fill(0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointVelocityController::on_activate(
  const rclcpp_lifecycle::State&)
{
  command_.fill(0.0);

  joint_velocity_subscriber_ =
    get_node()->create_subscription<
      std_msgs::msg::Float64MultiArray>(
        command_topic_,
        1,
        std::bind(
          &JointVelocityController::
            jointVelocityCommandCallback,
          this,
          std::placeholders::_1));

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Activated joint velocity controller for '%s' on '%s'",
    arm_id_.c_str(),
    command_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointVelocityController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  command_.fill(0.0);

  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  joint_velocity_subscriber_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointVelocityController::on_error(
  const rclcpp_lifecycle::State&)
{
  command_.fill(0.0);

  RCLCPP_ERROR(
    get_node()->get_logger(),
    "JointVelocityController entered error state");

  return CallbackReturn::ERROR;
}

void
JointVelocityController::
jointVelocityCommandCallback(
  const std_msgs::msg::Float64MultiArray& msg)
{
  if (msg.data.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Expected %zu joint velocities, received %zu",
      NUM_JOINTS,
      msg.data.size());

    return;
  }
  std::lock_guard<std::mutex> lock(command_mutex_);
  std::copy(
    msg.data.begin(),
    msg.data.end(),
    command_.begin());
}

}  // namespace panda
}  // namespace alberto_controllers
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  alberto_controllers::panda::JointVelocityController,
  controller_interface::ControllerInterface)