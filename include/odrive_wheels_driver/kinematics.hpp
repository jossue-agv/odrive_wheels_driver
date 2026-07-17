#pragma once

// Differential-drive inverse kinematics shared by the ODrive ROS 2 node.
//
// Units:
//   linear_x      m/s
//   angular_z     rad/s
//   wheel_radius  m
//   track_width   m
//   gear_ratio    motor turns per wheel turn

#include <cmath>

namespace odrive_wheels_driver {
namespace kinematics {

struct WheelVelocities {
  double left;   // motor turns/s
  double right;  // motor turns/s
};

inline WheelVelocities cmd_vel_to_wheels(
    double linear_x, double angular_z, double wheel_radius,
    double track_width, double gear_ratio) {
  const double left =
    (linear_x - angular_z * track_width / 2.0) /
    (wheel_radius * 2.0 * M_PI) * gear_ratio;
  const double right =
    (linear_x + angular_z * track_width / 2.0) /
    (wheel_radius * 2.0 * M_PI) * gear_ratio;
  return {left, right};
}

inline double motor_turns_to_wheel_radians(double motor_turns, double gear_ratio) {
  return motor_turns * 2.0 * M_PI / gear_ratio;
}

}  // namespace kinematics
}  // namespace odrive_wheels_driver
