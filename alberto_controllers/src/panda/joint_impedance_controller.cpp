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
/* CONTROLLER  */
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
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  
  updateJointStates();
  computeControl(period);
  writeCommands();
  publishDebugVelocities(time);

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

    auto_declare<double>("command_frequency",100.0);

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

  command_frequency_ =get_node()->get_parameter("command_frequency").as_double();

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


  if (command_frequency_ <= 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Parameter 'command_frequency' must be positive");

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

  dq_command_.setZero();
  dq_interpolated_.setZero();

  dq_interpolation_start_.setZero();
  dq_interpolation_end_.setZero();

  interpolation_elapsed_ = 0.0;
  new_command_received_ = false;

  debug_publishers_counter_ = 0;



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
  initializeDebugPublishers();
  q_goal_ = q_;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    dq_command_.setZero();
    dq_interpolated_.setZero();
    new_command_received_ = false;

    last_command_time_ = std::chrono::steady_clock::now();
  }



  dq_interpolation_start_.setZero();
  dq_interpolation_end_.setZero();



  interpolation_elapsed_ = 0.0;

  desired_torque_.setZero();

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

    dq_command_.setZero();
    dq_interpolated_.setZero();
    new_command_received_ = false;
  }



  dq_interpolation_start_.setZero();
  dq_interpolation_end_.setZero();

  interpolation_elapsed_ = 0.0;
  desired_torque_.setZero();

  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }
  dq_publisher.reset();

  dq_interpolated_publisher_.reset();

  debug_publishers_counter_ = 0;
  joint_velocity_subscriber_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn
JointImpedanceController::on_error(
  const rclcpp_lifecycle::State&)
{
  {
    std::lock_guard<std::mutex> lock(command_mutex_);

    dq_command_.setZero();
    dq_interpolated_.setZero();
    new_command_received_ = false;
  }


  dq_interpolation_start_.setZero();
  dq_interpolation_end_.setZero();



  interpolation_elapsed_ = 0.0;

  desired_torque_.setZero();
  for (auto& command_interface : command_interfaces_)
  {
    command_interface.set_value(0.0);
  }

  joint_velocity_subscriber_.reset();

  dq_publisher.reset();


  dq_interpolated_publisher_.reset();

  debug_publishers_counter_ = 0;

  RCLCPP_ERROR(get_node()->get_logger(),"JointImpedanceController entered error state");

  return CallbackReturn::ERROR;
}


/* IMPLEMENTATION */

void JointImpedanceController::initializeSubscriber()
{
  /*
   * Velocity command subscription:
   *
   * The controller runs at a higher frequency (e.g. 1 kHz) than the command
   * publisher (e.g. 100 Hz). For velocity commands, receiving an old command
   * is generally worse than losing a single message, because stale commands
   * introduce delay in the control loop.
   *
   * Therefore, use:
   * - KEEP_LAST(1): keep only the most recent command, discarding outdated data
   * - BEST_EFFORT: avoid DDS retransmissions of old commands that can introduce
   *   latency and affect real-time control behavior
   *
   * This QoS policy is suitable for real-time velocity/teleoperation commands,
   * where the priority is minimizing latency rather than guaranteeing delivery
   * of every message.
   */
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
  qos.best_effort();

  joint_velocity_subscriber_ =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      command_topic_,
      qos,
      std::bind(
        &JointImpedanceController::jointVelocityCommandCallback,
        this,
        std::placeholders::_1));
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

void JointImpedanceController::jointVelocityCommandCallback(
  const std_msgs::msg::Float64MultiArray& msg)
{
  if (msg.data.size() != NUM_JOINTS)
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Expected %zu velocity commands, received %zu",NUM_JOINTS,msg.data.size());
    return;
  }

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    dq_command_ = Eigen::Map<const Vector7d>(msg.data.data());

    new_command_received_ = true;
    last_command_time_ = std::chrono::steady_clock::now();
  }
}

Vector7d JointImpedanceController::interpolate(const Vector7d& start, const Vector7d& target ,double t)
{
  return (1.0 - t) * start + t * target;
   
}

void JointImpedanceController::computeControl( const rclcpp::Duration& period)
{
  const double dt = period.seconds();

  Vector7d command_target = Vector7d::Zero();
  bool new_command = false;

  double command_age = 0.0;
  bool command_timed_out = false;

  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (new_command_received_)
    {
      command_target = dq_command_;

      new_command = new_command_received_;

      new_command_received_ = false;

      command_age = std::chrono::duration<double>(std::chrono::steady_clock::now() - last_command_time_).count();

    }

   
  }

  if (command_age > command_timeout_)
  {
    command_target.setZero();
    command_timed_out = true;
  }

  if (new_command)
  {
    dq_interpolation_start_ = dq_interpolated_;
    dq_interpolation_end_   = command_target;
    interpolation_elapsed_ = 0.0;
    new_command = false;
    RCLCPP_WARN_THROTTLE( get_node()->get_logger(), *get_node()->get_clock(), 1000,
                        "Time since last command: %.3f s, new command received, interpolating to new target velocity",command_age);
   
  }


  

  const double alpha = std::clamp(interpolation_elapsed_ * command_frequency_,0.0,1.0);
  interpolation_elapsed_ += period.seconds();
  dq_interpolated_ = interpolate(dq_interpolation_start_,dq_interpolation_end_,alpha);



  /*
   * Local joint-position reference generated from the desired
   * joint velocity over the current control period.
   */


  q_goal_ += dq_interpolated_ * dt;

  const Vector7d position_error = q_goal_ - q_;

  const Vector7d velocity_error = dq_interpolated_ - dq_;
  
  desired_torque_ = k_gains_.cwiseProduct(position_error) + d_gains_.cwiseProduct(velocity_error);

}

void JointImpedanceController::writeCommands()
{
  

  for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
  {
    const double torque = desired_torque_(static_cast<Eigen::Index>(joint));

    command_interfaces_[joint].set_value( torque);

  }
}
void JointImpedanceController::initializeDebugPublishers()
{
  constexpr std::size_t queue_size = 10;

  dq_publisher =
    std::make_shared<RealtimeVelocityPublisher>(
      get_node()->create_publisher<
        std_msgs::msg::Float64MultiArray>(
          "~/debug/dq_command_target",
          queue_size));


  dq_interpolated_publisher_ =
    std::make_shared<RealtimeVelocityPublisher>(
      get_node()->create_publisher<
        std_msgs::msg::Float64MultiArray>(
          "~/debug/dq_interpolated",
          queue_size));

  dq_publisher->msg_.data.resize(NUM_JOINTS);
  dq_interpolated_publisher_->msg_.data.resize(NUM_JOINTS);
}
void JointImpedanceController::publishDebugVelocities( const rclcpp::Time& /*time*/)
{
  ++debug_publishers_counter_;

  if (debug_publishers_counter_ < debug_publish_decimation_)
  {
    return;
  }

  debug_publishers_counter_ = 0;

  if (
    dq_publisher &&
    dq_publisher->trylock())
  {
    for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
    {
      dq_publisher->msg_.data[joint] =
        dq_interpolation_end_(static_cast<Eigen::Index>(joint));
    }

    dq_publisher->unlockAndPublish();
  }

 

  if (
    dq_interpolated_publisher_ &&
    dq_interpolated_publisher_->trylock())
  {
    for (std::size_t joint = 0; joint < NUM_JOINTS; ++joint)
    {
      dq_interpolated_publisher_->msg_.data[joint] =
        dq_interpolated_(static_cast<Eigen::Index>(joint));
    }

    dq_interpolated_publisher_->unlockAndPublish();
  }
}


} // namespace panda
}  // namespace alberto_controllers

PLUGINLIB_EXPORT_CLASS(
  alberto_controllers::panda::JointImpedanceController,
  controller_interface::ControllerInterface)