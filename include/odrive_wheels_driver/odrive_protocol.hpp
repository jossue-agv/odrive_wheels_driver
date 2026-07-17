#pragma once

#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>

namespace odrive_wheels_driver {

// -- CAN arbitration ID encoding --
// ODrive flat endpoint: (node_id << 5) | cmd_id
inline uint32_t make_arb_id(uint8_t node_id, uint8_t cmd_id) {
  return (static_cast<uint32_t>(node_id) << 5) | cmd_id;
}

inline uint8_t get_node_id(uint32_t arb_id) {
  return static_cast<uint8_t>(arb_id >> 5);
}

inline uint8_t get_cmd_id(uint32_t arb_id) {
  return static_cast<uint8_t>(arb_id & 0x1F);
}

// -- CAN command IDs --
namespace cmd {
  constexpr uint8_t HEARTBEAT             = 0x01;
  constexpr uint8_t ESTOP                 = 0x02;
  constexpr uint8_t GET_ERROR             = 0x03;
  constexpr uint8_t RX_SDO                = 0x04;
  constexpr uint8_t SET_AXIS_STATE        = 0x07;
  constexpr uint8_t GET_ENCODER_ESTIMATES = 0x09;
  constexpr uint8_t SET_CONTROLLER_MODE   = 0x0B;
  constexpr uint8_t SET_INPUT_VEL         = 0x0D;
  constexpr uint8_t GET_TEMPERATURE       = 0x15;
  constexpr uint8_t REBOOT                = 0x16;
  constexpr uint8_t GET_VBUS_VOLTAGE      = 0x17;
  constexpr uint8_t CLEAR_ERRORS          = 0x18;
  constexpr uint8_t SET_VEL_GAINS         = 0x1B;
}

// -- Controller modes (Set_Controller_Mode, cmd 0x0B) --
namespace control_mode {
  constexpr uint32_t VOLTAGE  = 0;
  constexpr uint32_t TORQUE   = 1;
  constexpr uint32_t VELOCITY = 2;
  constexpr uint32_t POSITION = 3;
}

namespace input_mode {
  constexpr uint32_t INACTIVE    = 0;
  constexpr uint32_t PASSTHROUGH = 1;
  constexpr uint32_t VEL_RAMP    = 2;
}

// -- Axis states --
enum class AxisState : uint8_t {
  UNDEFINED              = 0,
  IDLE                   = 1,
  STARTUP_SEQUENCE       = 2,
  FULL_CALIBRATION       = 3,
  MOTOR_CALIBRATION      = 4,
  ENCODER_OFFSET_CALIB   = 7,
  CLOSED_LOOP_CONTROL    = 8,
};

// NOTA: JOSSUE con esto después hay que crear un sistema de alertas después
// -- Active error flags (32-bit bitmask) --
namespace error {
  constexpr uint32_t INITIALIZING               = 0x00000001;
  constexpr uint32_t SYSTEM_ERROR               = 0x00000002;
  constexpr uint32_t TIMING_ERROR               = 0x00000004;
  constexpr uint32_t MISSING_ESTIMATE           = 0x00000008;
  constexpr uint32_t BAD_CONFIG                 = 0x00000010;
  constexpr uint32_t DRV_FAULT                  = 0x00000020;
  constexpr uint32_t MISSING_INPUT              = 0x00000040;
  constexpr uint32_t DC_BUS_OVER_VOLTAGE        = 0x00000100;
  constexpr uint32_t DC_BUS_UNDER_VOLTAGE       = 0x00000200;
  constexpr uint32_t DC_BUS_OVER_CURRENT        = 0x00000400;
  constexpr uint32_t DC_BUS_OVER_REGEN_CURRENT  = 0x00000800;
  constexpr uint32_t CURRENT_LIMIT_VIOLATION    = 0x00001000;
  constexpr uint32_t MOTOR_OVER_TEMP            = 0x00002000;
  constexpr uint32_t INVERTER_OVER_TEMP         = 0x00004000;
  constexpr uint32_t VELOCITY_LIMIT_VIOLATION   = 0x00008000;
  constexpr uint32_t POSITION_LIMIT_VIOLATION   = 0x00010000;
  constexpr uint32_t WATCHDOG_TIMER_EXPIRED     = 0x01000000;
  constexpr uint32_t ESTOP_REQUESTED            = 0x02000000;
  constexpr uint32_t SPINOUT_DETECTED           = 0x04000000;
  constexpr uint32_t BRAKE_RESISTOR_DISARMED    = 0x08000000;
  constexpr uint32_t THERMISTOR_DISCONNECTED    = 0x10000000;
  constexpr uint32_t CALIBRATION_ERROR          = 0x40000000;
}


// -- Parsed CAN messages --
struct HeartbeatMsg {
  uint32_t active_errors;
  uint8_t  axis_state;
  uint8_t  procedure_result;
  uint8_t  trajectory_done;
  uint32_t disarm_reason;

  static HeartbeatMsg parse(const uint8_t* data) {
    HeartbeatMsg msg{};
    std::memcpy(&msg.active_errors, &data[0], 4);
    msg.axis_state = data[4];
    msg.procedure_result = data[5];
    msg.trajectory_done = data[6];
    std::memcpy(&msg.disarm_reason, &data[4], 4);
    return msg;
  }
};

struct EncoderMsg {
  float position;  // turns
  float velocity;  // turns/s
  bool valid;      // false if NaN/Inf detected

  static EncoderMsg parse(const uint8_t* data) {
    EncoderMsg msg{};
    std::memcpy(&msg.position, &data[0], 4);
    std::memcpy(&msg.velocity, &data[4], 4);
    msg.valid = std::isfinite(msg.position) && std::isfinite(msg.velocity);
    return msg;
  }
};

struct TemperatureMsg {
  float fet_temperature;    // °C
  float motor_temperature;  // °C

  static TemperatureMsg parse(const uint8_t* data) {
    TemperatureMsg msg{};
    std::memcpy(&msg.fet_temperature, &data[0], 4);
    std::memcpy(&msg.motor_temperature, &data[4], 4);
    return msg;
  }
};

struct VbusMsg {
  float voltage;  // V
  float current;  // A

  static VbusMsg parse(const uint8_t* data) {
    VbusMsg msg{};
    std::memcpy(&msg.voltage, &data[0], 4);
    std::memcpy(&msg.current, &data[4], 4);
    return msg;
  }
};

// -- Velocity command packing --
inline void pack_velocity(uint8_t* data, float velocity, float torque_ff = 0.0f) {
  std::memcpy(&data[0], &velocity, 4);
  std::memcpy(&data[4], &torque_ff, 4);
}

// -- Axis state command packing --
inline void pack_axis_state(uint8_t* data, AxisState state) {
  uint32_t s = static_cast<uint32_t>(state);
  std::memcpy(&data[0], &s, 4);
  std::memset(&data[4], 0, 4);
}

// -- Controller mode command packing (Control_Mode + Input_Mode, 2x uint32) --
inline void pack_controller_mode(uint8_t* data, uint32_t control_mode_val,
                                 uint32_t input_mode_val) {
  std::memcpy(&data[0], &control_mode_val, 4);
  std::memcpy(&data[4], &input_mode_val, 4);
}

// -- Velocity-loop PI gains (Set_Vel_Gains, cmd 0x1B) --
inline void pack_vel_gains(uint8_t* data, float vel_gain, float vel_integrator_gain) {
  std::memcpy(&data[0], &vel_gain, 4);
  std::memcpy(&data[4], &vel_integrator_gain, 4);
}

// ODrive S1 firmware 0.6.11 flat endpoint IDs.
namespace endpoint {
  constexpr uint16_t VEL_LIMIT = 374;
}

// -- RxSdo WRITE-float (cmd 0x04) --
inline void pack_rxsdo_write_float(uint8_t* data, uint16_t endpoint_id, float value) {
  data[0] = 0x01;
  std::memcpy(&data[1], &endpoint_id, 2);
  data[3] = 0x00;
  std::memcpy(&data[4], &value, 4);
}

}  // namespace odrive_wheels_driver
