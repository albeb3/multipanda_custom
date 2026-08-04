#include "alberto_controllers/panda/joint_impedance_controller.hpp"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <exception>
#include <functional>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

namespace alberto_controllers
{
namespace panda
{

controller_interface::InterfaceConfiguration
JointImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (std::size_t joint = 1; joint <= NUM_JOINTS; ++joint)
  {
    config.names.push_back( arm_id_ + "_joint" +std::to_string(joint) +"/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration
JointImpedanceController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;

  config.type =
    controller_interface::
      interface_configuration_type::INDIVIDUAL;

  for (std::size_t joint = 1; joint <= NUM_JOINTS; ++joint)
  {
    const std::string joint_name = arm_id_ + "_joint" + std::to_string(joint);

    config.names.push_back(joint_name + "/position");

    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::return_type
JointImpedanceController::update(
  const rclcpp::Time& /*time*/,
  const rclcpp::Duration& period)
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

  if (state_interfaces_.size() != 2 * NUM_JOINTS)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Expected %zu state interfaces, found %zu",
      2 * NUM_JOINTS,
      state_interfaces_.size());

    return controller_interface::return_type::ERROR;
  }

  updateJointStates();

  Vector7d desired_velocity = Vector7d::Zero();

  double command_age = 0.0;
  bool command_timed_out = false;

  {
    std::lock_guard<std::mutex> lock(
      command_mutex_);

    const auto now = std::chrono::steady_clock::now();

    command_age = std::chrono::duration<double>( now - last_command_time_).count();

    if (command_age > command_timeout_)
    {
      dq_desired_.setZero();
      command_timed_out = true;
    }

    desired_velocity = dq_desired_;
  }

  if (command_timed_out)
  {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Velocity command timeout: age=%.3f s, timeout=%.3f s",
      command_age,
      command_timeout_);
  }

  const double dt = period.seconds();

  if (dt <= 0.0)
  {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(),
      *get_node()->get_clock(),
      1000,
      "Controller period must be positive");

    return controller_interface::return_type::ERROR;
  }

  /*
   * Local joint-position reference generated from the desired
   * joint velocity over the current control period.
   */
  q_goal_ = q_ + desired_velocity * dt;

  dq_desired_filtered_ = velocity_filter_alpha_ * desired_velocity + (1.0 - velocity_filter_alpha_) * dq_desired_filtered_;

  dq_filtered_ = velocity_filter_alpha_ * dq_ + (1.0 - velocity_filter_alpha_) * dq_filtered_;

  const Vector7d position_error = q_goal_ - q_;

  const Vector7d velocity_error = dq_desired_filtered_ ;

  const Vector7d desired_torque = k_gains_.cwiseProduct(position_error) + d_gains_.cwiseProduct(velocity_error);

  for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
  {
    const double torque = desired_torque( static_cast<Eigen::Index>(joint));

    command_interfaces_[joint].set_value( torque);

  }

  return controller_interface::return_type::OK;
}

CallbackReturn
JointImpedanceController::on_init()
{
  try
  {
    auto_declare<std::string>("arm_id","panda");

    auto_declare<std::string>("command_topic","/joint_velocities");

    auto_declare<double>("command_timeout",0.5);

    auto_declare<double>("velocity_filter_alpha",0.99);

    auto_declare<std::vector<double>>("k_gains",{});

    auto_declare<std::vector<double>>("d_gains",{});
  }
  catch (const std::exception& exception)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Initialization failed: %s",exception.what());

    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_configure(
  const rclcpp_lifecycle::State&)
{
  arm_id_ =get_node()->get_parameter("arm_id").as_string();

  command_topic_ =get_node()->get_parameter("command_topic").as_string();

  command_timeout_ =get_node()->get_parameter("command_timeout").as_double();

  velocity_filter_alpha_ =get_node()->get_parameter("velocity_filter_alpha").as_double();

  const std::vector<double> k_gains =get_node()->get_parameter("k_gains").as_double_array();

  const std::vector<double> d_gains =get_node()->get_parameter("d_gains").as_double_array();

  if (arm_id_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'arm_id' cannot be empty");

    return CallbackReturn::FAILURE;
  }

  if (command_topic_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'command_topic' cannot be empty");

    return CallbackReturn::FAILURE;
  }

  if (command_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'command_timeout' must be positive");

    return CallbackReturn::FAILURE;
  }

  if (velocity_filter_alpha_ < 0.0 ||velocity_filter_alpha_ > 1.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'velocity_filter_alpha' must be in [0, 1]");

    return CallbackReturn::FAILURE;
  }

  if (k_gains.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'k_gains' must contain %zu values, received %zu",NUM_JOINTS,k_gains.size());

    return CallbackReturn::FAILURE;
  }

  if (d_gains.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'd_gains' must contain %zu values, received %zu",NUM_JOINTS,d_gains.size());

    return CallbackReturn::FAILURE;
  }

  for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
  {
    const Eigen::Index index = static_cast<Eigen::Index>(joint);

    k_gains_(index) = k_gains[joint];

    d_gains_(index) = d_gains[joint];
  }

  q_.setZero();
  dq_.setZero();
  q_goal_.setZero();

  dq_desired_.setZero();
  dq_desired_filtered_.setZero();
  dq_filtered_.setZero();

  RCLCPP_INFO(get_node()->get_logger(),"Configured joint impedance controller for arm '%s'",arm_id_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_activate(
  const rclcpp_lifecycle::State&)
{
  if (command_interfaces_.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Cannot activate: expected %zu command interfaces, found %zu",NUM_JOINTS,command_interfaces_.size());

    return CallbackReturn::FAILURE;
  }

  if (state_interfaces_.size() != 2 * NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Cannot activate: expected %zu state interfaces, found %zu",2 * NUM_JOINTS,state_interfaces_.size());

    return CallbackReturn::FAILURE;
  }

  updateJointStates();

  q_goal_ =
    q_;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    dq_desired_.setZero();

    last_command_time_ = std::chrono::steady_clock::now();
  }

  dq_desired_filtered_.setZero();
  dq_filtered_.setZero();

  initializeSubscriber();

  RCLCPP_INFO(get_node()->get_logger(),"=== COMMAND INTERFACES (%zu) ===",command_interfaces_.size());

  for (std::size_t i = 0; i < command_interfaces_.size(); ++i)
  {
    RCLCPP_INFO(get_node()->get_logger(),"CMD[%zu]: %s/%s",i,command_interfaces_[i].get_prefix_name().c_str(),command_interfaces_[i].get_interface_name().c_str());
  }

  RCLCPP_INFO(get_node()->get_logger(),"=== STATE INTERFACES (%zu) ===",state_interfaces_.size());

  for (std::size_t i = 0; i < state_interfaces_.size(); ++i)
  {
    RCLCPP_INFO(get_node()->get_logger(),"STATE[%zu]: %s/%s = %.8f",i,state_interfaces_[i].get_prefix_name().c_str(),state_interfaces_[i].get_interface_name().c_str(),state_interfaces_[i].get_value());
  }

  RCLCPP_INFO(get_node()->get_logger(),"Activated joint impedance controller for '%s' on topic '%s'",arm_id_.c_str(),command_topic_.c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    dq_desired_.setZero();
  }

  dq_desired_filtered_.setZero();
  dq_filtered_.setZero();

  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  joint_velocity_subscriber_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_error(
  const rclcpp_lifecycle::State&)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    dq_desired_.setZero();
  }

  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  RCLCPP_ERROR(get_node()->get_logger(),"JointImpedanceController entered error state");

  return CallbackReturn::ERROR;
}

void JointImpedanceController::updateJointStates()
{
  for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
  {
    const auto& position_interface = state_interfaces_.at(2 * joint);

    const auto& velocity_interface = state_interfaces_.at(2 * joint + 1);

    assert( position_interface.get_interface_name() == "position");

    assert(velocity_interface.get_interface_name() == "velocity");

    const Eigen::Index index = static_cast<Eigen::Index>(joint);

    q_(index) = position_interface.get_value();

    dq_(index) = velocity_interface.get_value();
  }
}

void JointImpedanceController::initializeSubscriber()
{
  joint_velocity_subscriber_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(command_topic_,1,std::bind(
          &JointImpedanceController::jointVelocityCommandCallback,this,std::placeholders::_1));
}

void JointImpedanceController::jointVelocityCommandCallback(
  const std_msgs::msg::Float64MultiArray& msg)
{
  if (msg.data.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Expected %zu velocity commands, received %zu",NUM_JOINTS,msg.data.size());

    return;
  }

  Vector7d received_command = Vector7d::Zero();

  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
    {
      const Eigen::Index index = static_cast<Eigen::Index>(joint);

      dq_desired_(index) = msg.data[joint];

      received_command(index) = msg.data[joint];
    }

    last_command_time_ = std::chrono::steady_clock::now();
  }

  RCLCPP_INFO_STREAM_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000, "\nReceived /joint_velocities:" "\n  dq desired = "<< received_command.transpose());
}

}  // namespace panda
}  // namespace alberto_controllers

PLUGINLIB_EXPORT_CLASS(
  alberto_controllers::panda::JointImpedanceController,
  controller_interface::ControllerInterface)