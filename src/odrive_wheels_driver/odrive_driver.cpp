#include "odrive_wheels_driver/odrive_driver.hpp"

#include <linux/can.h>
#include <utility>

namespace odrive_wheels_driver {

ODriveDriver::ODriveDriver(
    std::string can_interface, uint8_t left_axis_id, uint8_t right_axis_id)
    : can_interface_(std::move(can_interface)),
      left_axis_id_(left_axis_id),
      right_axis_id_(right_axis_id) {}

ODriveDriver::~ODriveDriver() {
  disconnect();
}

bool ODriveDriver::connect() {
  if (is_connected()) {
    return true;
  }

  can_ = std::make_unique<CANSocket>(can_interface_);
  if (!can_->is_open()) {
    can_.reset();
    return false;
  }

  can_->set_filter(left_axis_id_, right_axis_id_);
  const auto now = std::chrono::steady_clock::now();
  left_.last_feedback = now;
  right_.last_feedback = now;
  left_.last_encoder_feedback = now;
  right_.last_encoder_feedback = now;
  left_.encoder_valid = false;
  right_.encoder_valid = false;
  consecutive_send_failures_ = 0;
  return true;
}

void ODriveDriver::disconnect() {
  if (can_) {
    can_->close();
    can_.reset();
  }
}

bool ODriveDriver::is_connected() const {
  return can_ && can_->is_open();
}

int ODriveDriver::native_handle() const {
  return is_connected() ? can_->native_handle() : -1;
}

AxisFeedback* ODriveDriver::axis_for(uint8_t node_id) {
  if (node_id == left_axis_id_) {
    return &left_;
  }
  if (node_id == right_axis_id_) {
    return &right_;
  }
  return nullptr;
}

PollResult ODriveDriver::poll_one(int timeout_ms) {
  PollResult result;
  if (!is_connected()) {
    return result;
  }

  struct can_frame frame {};
  if (!can_->recv(frame, timeout_ms)) {
    return result;
  }
  result.frame_received = true;

  if (frame.can_id & CAN_RTR_FLAG) {
    return result;
  }

  const auto arbitration_id = frame.can_id & CAN_SFF_MASK;
  const uint8_t node_id = get_node_id(arbitration_id);
  const uint8_t command_id = get_cmd_id(arbitration_id);
  AxisFeedback* axis = axis_for(node_id);
  if (!axis) {
    return result;
  }

  switch (command_id) {
    case cmd::HEARTBEAT: {
      if (frame.can_dlc < 7) {
        break;
      }
      const auto heartbeat = HeartbeatMsg::parse(frame.data);
      axis->state = heartbeat.axis_state;
      axis->errors = heartbeat.active_errors;
      axis->last_feedback = std::chrono::steady_clock::now();
      break;
    }
    case cmd::GET_ENCODER_ESTIMATES: {
      if (frame.can_dlc < 8) {
        break;
      }
      const auto encoder = EncoderMsg::parse(frame.data);
      if (!encoder.valid) {
        result.invalid_encoder_frame = true;
        result.invalid_encoder_node_id = node_id;
        break;
      }
      const auto now = std::chrono::steady_clock::now();
      axis->position_turns = encoder.position;
      axis->velocity_turns_per_s = encoder.velocity;
      axis->encoder_valid = true;
      axis->last_feedback = now;
      axis->last_encoder_feedback = now;
      result.encoder_updated = true;
      result.encoder_node_id = node_id;
      break;
    }
    case cmd::GET_TEMPERATURE: {
      if (frame.can_dlc < 8) {
        break;
      }
      const auto temperature = TemperatureMsg::parse(frame.data);
      axis->fet_temperature = temperature.fet_temperature;
      axis->motor_temperature = temperature.motor_temperature;
      break;
    }
    case cmd::GET_VBUS_VOLTAGE: {
      if (frame.can_dlc < 8) {
        break;
      }
      const auto bus = VbusMsg::parse(frame.data);
      bus_voltage_ = bus.voltage;
      bus_current_ = bus.current;
      break;
    }
    default:
      break;
  }

  return result;
}

PollResult ODriveDriver::poll() {
  PollResult aggregate;
  while (true) {
    const auto result = poll_one(1);
    if (!result.frame_received) {
      break;
    }
    aggregate.frame_received = true;
    if (result.encoder_updated) {
      aggregate.encoder_updated = true;
      aggregate.encoder_node_id = result.encoder_node_id;
    }
    if (result.invalid_encoder_frame) {
      aggregate.invalid_encoder_frame = true;
      aggregate.invalid_encoder_node_id = result.invalid_encoder_node_id;
    }
  }
  return aggregate;
}

void ODriveDriver::request_encoder_estimates() {
  if (!is_connected()) {
    return;
  }
  can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_ENCODER_ESTIMATES));
  can_->send_rtr(make_arb_id(right_axis_id_, cmd::GET_ENCODER_ESTIMATES));
}

void ODriveDriver::request_diagnostics() {
  if (!is_connected()) {
    return;
  }
  can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_TEMPERATURE));
  can_->send_rtr(make_arb_id(right_axis_id_, cmd::GET_TEMPERATURE));
  can_->send_rtr(make_arb_id(left_axis_id_, cmd::GET_VBUS_VOLTAGE));
}

bool ODriveDriver::send_frame(uint32_t arb_id, const uint8_t* data) {
  if (!is_connected()) {
    return false;
  }
  if (can_->send(arb_id, data, 8)) {
    consecutive_send_failures_ = 0;
    return true;
  }
  ++consecutive_send_failures_;
  return false;
}

bool ODriveDriver::set_velocity(
    uint8_t node_id, float velocity_turns_per_s, float torque_ff) {
  uint8_t data[8];
  pack_velocity(data, velocity_turns_per_s, torque_ff);
  return send_frame(make_arb_id(node_id, cmd::SET_INPUT_VEL), data);
}

bool ODriveDriver::set_axis_state(uint8_t node_id, AxisState state) {
  uint8_t data[8];
  pack_axis_state(data, state);
  return send_frame(make_arb_id(node_id, cmd::SET_AXIS_STATE), data);
}

bool ODriveDriver::set_controller_mode(
    uint8_t node_id, uint32_t control_mode, uint32_t input_mode) {
  uint8_t data[8];
  pack_controller_mode(data, control_mode, input_mode);
  return send_frame(make_arb_id(node_id, cmd::SET_CONTROLLER_MODE), data);
}

bool ODriveDriver::set_velocity_gains(
    uint8_t node_id, float vel_gain, float vel_integrator_gain) {
  uint8_t data[8];
  pack_vel_gains(data, vel_gain, vel_integrator_gain);
  return send_frame(make_arb_id(node_id, cmd::SET_VEL_GAINS), data);
}

bool ODriveDriver::set_velocity_limit(uint8_t node_id, float vel_limit) {
  uint8_t data[8];
  pack_rxsdo_write_float(data, endpoint::VEL_LIMIT, vel_limit);
  return send_frame(make_arb_id(node_id, cmd::RX_SDO), data);
}

void ODriveDriver::stop_and_idle() {
  set_velocity(left_axis_id_, 0.0f);
  set_velocity(right_axis_id_, 0.0f);
  set_axis_state(left_axis_id_, AxisState::IDLE);
  set_axis_state(right_axis_id_, AxisState::IDLE);
}

const AxisFeedback& ODriveDriver::left() const {
  return left_;
}

const AxisFeedback& ODriveDriver::right() const {
  return right_;
}

double ODriveDriver::bus_voltage() const {
  return bus_voltage_;
}

double ODriveDriver::bus_current() const {
  return bus_current_;
}

int ODriveDriver::consecutive_send_failures() const {
  return consecutive_send_failures_;
}

}  // namespace odrive_wheels_driver
