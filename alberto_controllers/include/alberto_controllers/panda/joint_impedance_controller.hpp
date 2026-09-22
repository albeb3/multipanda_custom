#ifndef ALBERTO_CONTROLLERS_PANDA_JOINT_IMPEDANCE_CONTROLLER_HPP_
#define ALBERTO_CONTROLLERS_PANDA_JOINT_IMPEDANCE_CONTROLLER_HPP_

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Core>

#include <controller_interface/controller_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

#include <realtime_tools/realtime_publisher.h>

  using Vector7d =
    Eigen::Matrix<double, 7, 1>;

namespace alberto_controllers
{
  namespace panda
  {

    using CallbackReturn =rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class JointImpedanceController final : public controller_interface::ControllerInterface
    {
    public:
      static constexpr std::size_t NUM_JOINTS = 7;



      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

      CallbackReturn on_init() override;

      CallbackReturn on_configure( const rclcpp_lifecycle::State& previous_state) override;

      CallbackReturn on_activate( const rclcpp_lifecycle::State& previous_state) override;

      CallbackReturn on_deactivate( const rclcpp_lifecycle::State& previous_state) override;

      CallbackReturn on_error( const rclcpp_lifecycle::State& previous_state) override;

    private:
      void updateJointStates();

      void initializeSubscriber();

      void jointVelocityCommandCallback( const std_msgs::msg::Float64MultiArray& msg );

      static Vector7d interpolate( const Vector7d& start, const Vector7d& target, double alpha);

      void computeControl( const rclcpp::Duration& period);

      void writeCommands();

      void initializeDebugPublishers();

      void publishDebugVelocities(const rclcpp::Time& time);

      using RealtimeVelocityPublisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>;

      std::shared_ptr<RealtimeVelocityPublisher> dq_publisher;
      std::shared_ptr<RealtimeVelocityPublisher> dq_interpolated_publisher_;
      std::size_t debug_publishers_counter_{0};
      std::size_t debug_publish_decimation_{1};

      std::string arm_id_;
      std::string command_topic_;

      double command_timeout_{0.5};



      /*
      * Frequenza nominale del publisher dei comandi.
      * Determina la durata dell'interpolazione:
      *
      *   T_command = 1 / command_frequency_
      */
      double command_frequency_{100.0};

      Vector7d q_{Vector7d::Zero()};
      Vector7d dq_{Vector7d::Zero()};

      Vector7d q_goal_{Vector7d::Zero()};
      
      Vector7d dq_command_{Vector7d::Zero()};
      Vector7d dq_interpolated_{Vector7d::Zero()};

      /*
      * Estremi fissi della rampa corrente.
      */
      Vector7d dq_interpolation_start_{Vector7d::Zero()};
      Vector7d dq_interpolation_end_{Vector7d::Zero()};

      Vector7d k_gains_{Vector7d::Zero()};
      Vector7d d_gains_{Vector7d::Zero()};

      Vector7d desired_torque_{Vector7d::Zero()};
      Vector7d dq_command_snapshot_ {Vector7d::Zero()};
      /*
      * Tempo trascorso dall'inizio della rampa corrente.
      */
      double interpolation_elapsed_{0.0};

      /*
      * Scritto dalla callback e consumato dal ciclo di controllo.
      */
      bool new_command_received_{false};

      std::chrono::steady_clock::time_point last_command_time_{};
      std::mutex command_mutex_;

      rclcpp::Subscription<
        std_msgs::msg::Float64MultiArray>::SharedPtr
        joint_velocity_subscriber_;
    };

  }  // namespace panda
}  // namespace alberto_controllers

#endif