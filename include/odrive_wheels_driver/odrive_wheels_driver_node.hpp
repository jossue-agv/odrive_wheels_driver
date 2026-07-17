#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "odrive_wheels_driver/odrive_driver.hpp"

namespace odrive_wheels_driver {

class ODriveWheelsDriverNode : public rclcpp::Node {
public:
  ODriveWheelsDriverNode();
  ~ODriveWheelsDriverNode() override;
  bool is_can_connected() const;

private:
  // -- Parameters --
  std::string can_interface_;
  uint8_t left_axis_id_;
  uint8_t right_axis_id_;
  double wheel_radius_;
  double track_width_;
  int publish_rate_hz_;
  int cmd_vel_timeout_ms_;
  bool invert_left_;
  bool invert_right_;
  double left_sign_;    // +1.0 or -1.0
  double right_sign_;
  double left_scale_;
  double right_scale_;
  float min_effective_vel_;
  float stiction_torque_ff_;
  float max_wheel_accel_;
  float max_wheel_decel_;
  float zero_vel_epsilon_;
  float vel_gain_;
  float vel_integrator_gain_;
  float vel_limit_;
  double gear_ratio_;  // motor_turns / wheel_turns (see odrive_wheels_driver_node.cpp for docs)
  double max_fet_temp_;
  double max_motor_temp_;
  double critical_temp_offset_;

  // -- Standalone driver library --
  std::unique_ptr<ODriveDriver> driver_;
  bool init_driver();
  void read_can_messages();
  void send_velocity(uint8_t node_id, float vel_turns_per_s);
  void send_velocity_with_ff(uint8_t node_id, float vel_turns_per_s, float torque_ff);
  float apply_wheel_shaping(float target, float& prev_cmd, double scale, float dt);
  void send_axis_state(uint8_t node_id, AxisState state);
  void stop_motors();

  // -- ROS-side control and safety state --
  float left_prev_cmd_ = 0.0f;
  float right_prev_cmd_ = 0.0f;
  int left_thermal_level_ = 0;
  int right_thermal_level_ = 0;

  // -- Feedback watchdog --
  int feedback_timeout_ms_;   // 0 disables
  bool feedback_ok_ = true;

  void publish_joint_states();

  // -- E-stop --
  bool e_stop_active_ = false;

  // -- cmd_vel timeout --
  rclcpp::Time last_cmd_vel_time_;

  int diag_counter_ = 0;

  // -- Temperature monitoring --
  void check_temperature(float fet_temp, float motor_temp, int& thermal_level);
  std::string thermal_state_{"ok"};

  // -- CAN retry backoff --
  int can_retry_delay_ms_{100};
  rclcpp::Time last_can_retry_{0, 0, RCL_ROS_TIME};

  // -- Motor readiness --
  bool motors_armed() const;
  void publish_motor_state();
  void on_motor_enable(const std_msgs::msg::Bool& msg);

  // -- Publishers --
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_motor_state_;

  // -- Subscribers --
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_e_stop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_motor_enable_;

  // -- Callbacks --
  void on_cmd_vel(const geometry_msgs::msg::Twist& msg);
  void on_e_stop(const std_msgs::msg::Bool& msg);

  // -- Timers --
  rclcpp::TimerBase::SharedPtr timer_main_;
  rclcpp::TimerBase::SharedPtr timer_encoder_;
  rclcpp::TimerBase::SharedPtr timer_motor_state_;

  void main_loop();
  void encoder_request_loop();
};

}  // namespace odrive_wheels_driver
