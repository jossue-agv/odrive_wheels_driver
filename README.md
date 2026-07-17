# odrive_wheels_driver

Production C++17 ROS2 driver for ODrive S1 motor controllers over SocketCAN.
It converts `cmd_vel` into motor velocity commands and publishes raw encoder feedback and motor diagnostics.

> [!IMPORTANT]
> Estas son notas de Jossue Espinoza
> Mejoras futuras, el driver necesitará un estandar para publicar el error y necesitaremos un nodo blackbox
> Se necesita integrar un sistema de auto recuperación
> Hace generar el sistema de calibración automática, el fabricante menciona que se debe hacer por USB
> Pero existe un método a medias preparado por CAN que sería bueno implementar, el original no se terminó 
> Mantuve en scripts check_odrive_config por si acaso
> Agregué confirmaciones de puerto CAN para fallar temprano

## Nodes

- **odrive_wheels_driver_node** (C++17): ROS 2 wrapper for topics, parameters, command shaping, and safety policy.

## Architecture

- `include/odrive_wheels_driver/odrive_driver.hpp` + `src/odrive_wheels_driver/odrive_driver.cpp`:
  standalone C++ library for ODrive S1 CAN communication, commands, encoder feedback, and errors.
- `include/odrive_wheels_driver/can_socket.hpp` + `src/can_socket.cpp`: SocketCAN transport used by the library.
- `include/odrive_wheels_driver/odrive_protocol.hpp`: CANSimple message parsing and packing.
- `src/odrive_wheels_driver_node.cpp`: upper ROS 2 layer and executable entry point that consumes the standalone library.

## Dev Tools (Python, in `scripts/` — commissioning only, never in the robot runtime)

- `teleop_keyboard` — Minimal keyboard teleoperation publisher for `cmd_vel`
- `check_odrive_config` — ODrive S1 tuning config checker (USB)

Installed utilities are launched through ROS 2:

```bash
ros2 run odrive_wheels_driver teleop_keyboard
ros2 run odrive_wheels_driver check_odrive_config
ros2 run odrive_wheels_driver check_odrive_config --apply
```

## Topics

**Published:**
- `joint_states` (sensor_msgs/JointState, 50 Hz) — Wheel angles and velocities after direction and gearbox correction (radians)
- `motor_state` (std_msgs/String, 10 Hz) — JSON: axis state, errors, voltage, temperatures, `feedback_ok`

**Subscribed:**
- `cmd_vel` (geometry_msgs/Twist) — Linear x + angular z velocity commands
- `e_stop` (std_msgs/Bool) — Emergency stop (true = immediate stop)
- `motor_enable` (std_msgs/Bool) — Arm/disarm motors (CLOSED_LOOP_CONTROL / IDLE)

## Parameters

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `can_interface` | `"can0"` | SocketCAN interface |
| `left_axis_id` | `0` | CAN node ID left motor |
| `right_axis_id` | `1` | CAN node ID right motor |
| `wheel_radius` | *(required, from geometry config)* | Used only to convert `cmd_vel` to motor turns/s |
| `track_width` | *(required, from geometry config)* | Used only to split angular velocity between wheels |
| `gear_ratio` | *(required, from geometry config)* | Motor turns per wheel turn |
| `publish_rate_hz` | `50` | Main loop frequency |
| `cmd_vel_timeout_ms` | `200` | Stop motors if no `cmd_vel` arrives |
| `feedback_timeout_ms` | `300` | Feedback watchdog: fault if no heartbeat/encoder frame for this long (0 disables) |
| `invert_left` | `true` | Negate left motor direction |
| `invert_right` | `false` | Negate right motor direction |
| `left_scale` / `right_scale` | `1.0` | Per-wheel velocity trim |
| `max_wheel_accel` | `0.625` | Acceleration rate limiter (turns/s²) |
| `max_wheel_decel` | `1.875` | Deceleration rate limiter (turns/s²) |
| `zero_vel_epsilon` | `0.03` | Velocity deadband (turns/s) |
| `min_effective_vel` | `0.0` | Stiction compensation minimum |
| `stiction_torque_ff` | `0.03` | Torque feedforward (Nm) |

## Key Algorithms

- **Feedback watchdog**: Reports motors disarmed and zeroes encoder velocities when heartbeat/encoder feedback expires
- **Inverse kinematics**: `v_left/right = (linear_x -/+ angular_z * track_width/2) / (wheel_radius * 2pi) * gear_ratio`
- **Wheel shaping pipeline**: Zero bypass -> asymmetric accel/decel limiter -> stiction compensation -> torque feedforward
- **CAN protocol**: Arbitration ID = `(node_id << 5) | cmd_id`, commands: HEARTBEAT, GET_ENCODER_ESTIMATES, SET_INPUT_VEL, SET_AXIS_STATE, GET_TEMPERATURE, GET_VBUS_VOLTAGE

## Configuration

- `config/odrive_params.yaml` — All parameters above
- `launch/odrive_wheels_driver.launch.py` — Supports namespace, params_file, cmd_vel_topic remapping

## Dependencies

- SocketCAN (Linux kernel), rclcpp, sensor_msgs, geometry_msgs, std_msgs

## Improvement Opportunities

- Enable and validate the ODrive firmware axis watchdog (`axis.config.enable_watchdog`)
  so a host crash disarms the motors controller-side (needs hardware validation)
- Add integration test with mock CAN socket (RTR-frame regression test)
- Add docstrings to Python dev tools
- Validate gear_ratio against ODrive firmware config to prevent silent misconfiguration
