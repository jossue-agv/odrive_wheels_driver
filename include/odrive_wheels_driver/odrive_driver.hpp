#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include "odrive_wheels_driver/can_socket.hpp"
#include "odrive_wheels_driver/odrive_protocol.hpp"

namespace odrive_wheels_driver {

struct AxisFeedback {
  float position_turns = 0.0f;
  float velocity_turns_per_s = 0.0f;
  float fet_temperature = 0.0f;
  float motor_temperature = 0.0f;
  uint8_t state = 0;
  uint32_t errors = 0;
  bool encoder_valid = false;
  std::chrono::steady_clock::time_point last_feedback{};
};

struct PollResult {
  bool invalid_encoder_frame = false;
  uint8_t invalid_encoder_node_id = 0;
};

// Standalone ODrive S1 CANSimple interface. This class contains no ROS code.
class ODriveDriver {
public:
  ODriveDriver(std::string can_interface, uint8_t left_axis_id, uint8_t right_axis_id);
  ~ODriveDriver();

  ODriveDriver(const ODriveDriver&) = delete;
  ODriveDriver& operator=(const ODriveDriver&) = delete;

  bool connect();
  void disconnect();
  bool is_connected() const;

  PollResult poll();
  void request_encoder_estimates();
  void request_diagnostics();

  bool set_velocity(uint8_t node_id, float velocity_turns_per_s, float torque_ff = 0.0f);
  bool set_axis_state(uint8_t node_id, AxisState state);
  bool set_velocity_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);
  bool set_velocity_limit(uint8_t node_id, float vel_limit);
  void stop_and_idle();

  const AxisFeedback& left() const;
  const AxisFeedback& right() const;
  double bus_voltage() const;
  double bus_current() const;
  int consecutive_send_failures() const;

private:
  AxisFeedback* axis_for(uint8_t node_id);
  bool send_frame(uint32_t arb_id, const uint8_t* data);

  std::string can_interface_;
  uint8_t left_axis_id_;
  uint8_t right_axis_id_;
  std::unique_ptr<CANSocket> can_;
  AxisFeedback left_;
  AxisFeedback right_;
  double bus_voltage_ = 0.0;
  double bus_current_ = 0.0;
  int consecutive_send_failures_ = 0;
};

}  // namespace odrive_wheels_driver
