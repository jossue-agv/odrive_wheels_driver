#include "odrive_wheels_driver/odrive_wheels_driver_node.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "rclcpp/exceptions.hpp"

#include "odrive_wheels_driver/kinematics.hpp"

namespace {

class GuardConditionCallbackWaitable : public rclcpp::Waitable {
public:
  GuardConditionCallbackWaitable(
      rclcpp::GuardCondition::SharedPtr guard_condition,
      std::function<void()> callback)
      : guard_condition_(std::move(guard_condition)),
        callback_(std::move(callback)) {}

  size_t get_number_of_ready_guard_conditions() override {
    return 1U;
  }

  void add_to_wait_set(rcl_wait_set_t* wait_set) override {
    const rcl_ret_t result = rcl_wait_set_add_guard_condition(
      wait_set, &guard_condition_->get_rcl_guard_condition(), nullptr);
    if (result != RCL_RET_OK) {
      rclcpp::exceptions::throw_from_rcl_error(
        result, "failed to add CAN guard condition to wait set");
    }
  }

  bool is_ready(rcl_wait_set_t* wait_set) override {
    const auto* guard_condition =
      &guard_condition_->get_rcl_guard_condition();
    for (size_t index = 0; index < wait_set->size_of_guard_conditions; ++index) {
      if (wait_set->guard_conditions[index] == guard_condition) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<void> take_data() override {
    return nullptr;
  }

  void execute(std::shared_ptr<void>& data) override {
    (void)data;
    callback_();
  }

private:
  rclcpp::GuardCondition::SharedPtr guard_condition_;
  std::function<void()> callback_;
};

}  // namespace

namespace odrive_wheels_driver {

ODriveWheelsDriverNode::ODriveWheelsDriverNode() : Node("odrive_wheels_driver_node") {
  // -- Declare parameters --
  this->declare_parameter("can_interface", "can0");
  this->declare_parameter("left_axis_id", 1);
  this->declare_parameter("right_axis_id", 0);
  // Calibrated geometry for the current 10:1 drivetrain.
  this->declare_parameter("wheel_radius", 0.0625);
  this->declare_parameter("track_width", 0.720);
  this->declare_parameter("publish_rate_hz", 50);
  this->declare_parameter("cmd_vel_timeout_ms", 200);
  this->declare_parameter("invert_left", true);
  this->declare_parameter("invert_right", false);
  this->declare_parameter("left_scale", 1.0);
  this->declare_parameter("right_scale", 1.0);
  this->declare_parameter("min_effective_vel", 0.0);
  this->declare_parameter("stiction_torque_ff", 0.03);
  this->declare_parameter("max_wheel_accel", 10.0);
  this->declare_parameter("max_wheel_decel", 10.0);
  this->declare_parameter("zero_vel_epsilon", 0.03);
  this->declare_parameter("vel_gain", 0.23);
  this->declare_parameter("vel_integrator_gain", 0.23);
  this->declare_parameter("vel_limit", 14.0);
  // gear_ratio = motor turns / wheel turn. ODrive CAN feedback is interpreted
  // as raw motor turns, so the 10:1 reduction is applied exactly once here.
  this->declare_parameter("gear_ratio", 10.0);
  this->declare_parameter("max_fet_temp", 70.0);
  this->declare_parameter("max_motor_temp", 80.0);
  this->declare_parameter("critical_temp_offset", 10.0);
  // Feedback watchdog: fault if no heartbeat/encoder frame arrives within this
  // window (ODrive heartbeat period is 100 ms → default = 3 missed). 0 disables.
  this->declare_parameter("feedback_timeout_ms", 300);

  // -- Read parameters --
  can_interface_ = this->get_parameter("can_interface").as_string();
  left_axis_id_ = static_cast<uint8_t>(this->get_parameter("left_axis_id").as_int());
  right_axis_id_ = static_cast<uint8_t>(this->get_parameter("right_axis_id").as_int());
  wheel_radius_ = this->get_parameter("wheel_radius").as_double();
  track_width_ = this->get_parameter("track_width").as_double();
  gear_ratio_ = this->get_parameter("gear_ratio").as_double();
  publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_int();
  cmd_vel_timeout_ms_ = this->get_parameter("cmd_vel_timeout_ms").as_int();
  invert_left_ = this->get_parameter("invert_left").as_bool();
  invert_right_ = this->get_parameter("invert_right").as_bool();
  left_sign_ = invert_left_ ? -1.0 : 1.0;
  right_sign_ = invert_right_ ? -1.0 : 1.0;
  left_scale_ = this->get_parameter("left_scale").as_double();
  right_scale_ = this->get_parameter("right_scale").as_double();
  min_effective_vel_ = static_cast<float>(this->get_parameter("min_effective_vel").as_double());
  stiction_torque_ff_ = static_cast<float>(this->get_parameter("stiction_torque_ff").as_double());
  max_wheel_accel_ = static_cast<float>(this->get_parameter("max_wheel_accel").as_double());
  max_wheel_decel_ = static_cast<float>(this->get_parameter("max_wheel_decel").as_double());
  zero_vel_epsilon_ = static_cast<float>(this->get_parameter("zero_vel_epsilon").as_double());
  vel_gain_ = static_cast<float>(this->get_parameter("vel_gain").as_double());
  vel_integrator_gain_ = static_cast<float>(this->get_parameter("vel_integrator_gain").as_double());
  vel_limit_ = static_cast<float>(this->get_parameter("vel_limit").as_double());
  max_fet_temp_ = this->get_parameter("max_fet_temp").as_double();
  max_motor_temp_ = this->get_parameter("max_motor_temp").as_double();
  critical_temp_offset_ = this->get_parameter("critical_temp_offset").as_double();
  feedback_timeout_ms_ = static_cast<int>(this->get_parameter("feedback_timeout_ms").as_int());

  // -- Validate --
  if (wheel_radius_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "wheel_radius must be > 0, got %f", wheel_radius_);
    throw std::runtime_error("Invalid wheel_radius");
  }
  if (track_width_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "track_width must be > 0, got %f", track_width_);
    throw std::runtime_error("Invalid track_width");
  }
  if (gear_ratio_ <= 0.0) {
    RCLCPP_FATAL(get_logger(), "gear_ratio must be > 0, got %f", gear_ratio_);
    throw std::runtime_error("Invalid gear_ratio");
  }
  if (publish_rate_hz_ < 1 || publish_rate_hz_ > 1000) {
    RCLCPP_FATAL(get_logger(), "publish_rate_hz must be in [1, 1000], got %d", publish_rate_hz_);
    throw std::runtime_error("Invalid publish_rate_hz");
  }
  if (feedback_timeout_ms_ < 0) {
    RCLCPP_FATAL(get_logger(), "feedback_timeout_ms must be >= 0 (0 disables), got %d",
                 feedback_timeout_ms_);
    throw std::runtime_error("Invalid feedback_timeout_ms");
  }

  // -- Kinematics boot diagnostic (CRITICAL-02-02 closure) --
  // Log the live kinematics at every boot so the operator can cross-check
  // against the geometry SSOT (agv_description/config/robot_geometry.yaml)
  // and, on any doubt, against ODrive NVRAM via odrivetool
  // (docs/calibration/odrive_nvram_dump_procedure.md). If ROS gear_ratio
  // != 1.0 AND ODrive NVRAM also gears, motor turns are double-counted —
  // verify before deployment.
  RCLCPP_INFO(get_logger(),
              "Kinematics SSOT: wheel_radius=%.4fm, track_width=%.4fm, gear_ratio=%.2f. "
              "Cross-check against robot_geometry.yaml and ODrive NVRAM "
              "(encoder.cpr, motor.config.pole_pairs) per "
              "docs/calibration/odrive_nvram_dump_procedure.md.",
              wheel_radius_, track_width_, gear_ratio_);

  // -- Publishers --
  pub_joint_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  pub_motor_state_ = this->create_publisher<std_msgs::msg::String>("motor_state", 10);

  // -- Subscribers --
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", 10, std::bind(&ODriveWheelsDriverNode::on_cmd_vel, this, std::placeholders::_1));
  sub_e_stop_ = this->create_subscription<std_msgs::msg::Bool>(
    "e_stop", 10, std::bind(&ODriveWheelsDriverNode::on_e_stop, this, std::placeholders::_1));
  sub_motor_enable_ = this->create_subscription<std_msgs::msg::Bool>(
    "motor_enable", 10, std::bind(&ODriveWheelsDriverNode::on_motor_enable, this, std::placeholders::_1));

  // -- Initialize standalone driver library --
  driver_ = std::make_unique<ODriveDriver>(
    can_interface_, left_axis_id_, right_axis_id_);
  if (!init_driver()) {
    RCLCPP_ERROR(get_logger(), "Failed to open CAN interface %s. Will retry in timers.",
                 can_interface_.c_str());
  }

  last_cmd_vel_time_ = this->now();

  // -- Timers --
  auto main_period = std::chrono::milliseconds(1000 / publish_rate_hz_);
  timer_main_ = this->create_wall_timer(main_period,
    std::bind(&ODriveWheelsDriverNode::main_loop, this));

  // Encoder estimates are streamed cyclically by each ODrive every 10 ms.
  // This node only consumes those frames; it does not poll them with RTR.
  timer_diagnostics_ = this->create_wall_timer(std::chrono::seconds(1),
    std::bind(&ODriveWheelsDriverNode::diagnostics_request_loop, this));

  // Motor state at 10 Hz (100ms) — higher frequency ensures reliable DDS
  // discovery between C++ and rclnodejs subscribers on same machine.
  timer_motor_state_ = this->create_wall_timer(std::chrono::milliseconds(100),
    std::bind(&ODriveWheelsDriverNode::publish_motor_state, this));

  RCLCPP_INFO(get_logger(), "ODrive CAN node started on %s (left=%d, right=%d)",
              can_interface_.c_str(), left_axis_id_, right_axis_id_);
  RCLCPP_INFO(get_logger(), "wheel_radius=%.4f m, track_width=%.4f m, rate=%d Hz, gear_ratio=%.2f",
              wheel_radius_, track_width_, publish_rate_hz_, gear_ratio_);
  RCLCPP_INFO(get_logger(), "invert_left=%s, invert_right=%s",
              invert_left_ ? "true" : "false", invert_right_ ? "true" : "false");

  if (driver_->is_connected()) {
    start_can_event_loop();
  }
}

ODriveWheelsDriverNode::~ODriveWheelsDriverNode() {
  stop_can_event_loop();
  stop_motors();
}

bool ODriveWheelsDriverNode::is_can_connected() const {
  return driver_ && driver_->is_connected();
}

// ── Driver initialization ──

bool ODriveWheelsDriverNode::init_driver() {
  if (!driver_->connect()) {
    return false;
  }
  RCLCPP_INFO(get_logger(), "CAN socket opened on %s", can_interface_.c_str());

  const auto fail_initialization = [this](const char* reason) {
    RCLCPP_ERROR(get_logger(), "%s", reason);
    driver_->stop_and_idle();
    driver_->disconnect();
    return false;
  };

  const bool left_disabled_ok = driver_->set_axis_state(left_axis_id_, AxisState::IDLE);
  const bool right_disabled_ok = driver_->set_axis_state(right_axis_id_, AxisState::IDLE);
  if (!left_disabled_ok || !right_disabled_ok) {
    return fail_initialization("Failed to disable both ODrive axes during initialization");
  }
  RCLCPP_INFO(get_logger(), "ODrive axes disabled for safe initialization");

  const bool left_gains_ok = driver_->set_velocity_gains(
    left_axis_id_, vel_gain_, vel_integrator_gain_);
  const bool right_gains_ok = driver_->set_velocity_gains(
    right_axis_id_, vel_gain_, vel_integrator_gain_);
  const bool left_limit_ok = driver_->set_velocity_limit(left_axis_id_, vel_limit_);
  const bool right_limit_ok = driver_->set_velocity_limit(right_axis_id_, vel_limit_);

  if (!left_gains_ok || !right_gains_ok || !left_limit_ok || !right_limit_ok) {
    return fail_initialization("Failed to configure ODrive velocity gains or limits");
  }
  applied_vel_limit_ = vel_limit_;
  RCLCPP_INFO(
    get_logger(),
    "Configured velocity gains (%.3f, %.3f) and limit %.3f turns/s",
    static_cast<double>(vel_gain_), static_cast<double>(vel_integrator_gain_),
    static_cast<double>(vel_limit_));

  const bool left_mode_ok = driver_->set_controller_mode(
    left_axis_id_, control_mode::VELOCITY, input_mode::PASSTHROUGH);
  const bool right_mode_ok = driver_->set_controller_mode(
    right_axis_id_, control_mode::VELOCITY, input_mode::PASSTHROUGH);
  if (!left_mode_ok || !right_mode_ok) {
    return fail_initialization("Failed to configure ODrive controller mode");
  }
  RCLCPP_INFO(get_logger(), "Configured ODrive controller mode: VELOCITY + PASSTHROUGH");

  const bool left_zero_ok = driver_->set_velocity(left_axis_id_, 0.0F);
  const bool right_zero_ok = driver_->set_velocity(right_axis_id_, 0.0F);
  if (!left_zero_ok || !right_zero_ok) {
    return fail_initialization("Failed to set zero velocity before enabling ODrive axes");
  }

  if (!validate_cyclic_encoder_feedback()) {
    return fail_initialization("Cyclic encoder feedback validation failed");
  }

  const bool left_enabled_ok = driver_->set_axis_state(
    left_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
  const bool right_enabled_ok = driver_->set_axis_state(
    right_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
  if (!left_enabled_ok || !right_enabled_ok) {
    return fail_initialization("Failed to enable both ODrive axes after initialization");
  }
  RCLCPP_INFO(get_logger(), "ODrive axes enabled in CLOSED_LOOP_CONTROL");

  return true;
}

bool ODriveWheelsDriverNode::validate_cyclic_encoder_feedback() {
  constexpr int kMinimumStartupTimeoutMs = 500;
  constexpr int kAutobaudBeaconPeriodMs = 100;
  const int startup_timeout_ms =
    std::max(kMinimumStartupTimeoutMs, feedback_timeout_ms_);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(startup_timeout_ms);

  bool left_seen = false;
  bool right_seen = false;

  while (std::chrono::steady_clock::now() < deadline &&
         (!left_seen || !right_seen)) {
    const auto now = std::chrono::steady_clock::now();
    const int remaining_ms = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    const auto result = driver_->poll_one(
      std::max(1, std::min(kAutobaudBeaconPeriodMs, remaining_ms)));

    if (result.encoder_updated) {
      left_seen = left_seen || result.encoder_node_id == left_axis_id_;
      right_seen = right_seen || result.encoder_node_id == right_axis_id_;
    }
    if (result.invalid_encoder_frame) {
      RCLCPP_WARN(
        get_logger(), "ODrive node %u sent invalid cyclic encoder feedback",
        static_cast<unsigned>(result.invalid_encoder_node_id));
    }

    if (!result.frame_received) {
      // Safe zero-velocity frames also serve as the 10 Hz beacon recommended
      // for CAN autobaud on firmware 0.6.11+.
      driver_->set_velocity(left_axis_id_, 0.0F);
      driver_->set_velocity(right_axis_id_, 0.0F);
    }
  }

  if (!left_seen || !right_seen) {
    RCLCPP_ERROR(
      get_logger(),
      "Missing cyclic Get_Encoder_Estimates (0x09): left=%s right=%s. "
      "Set axis0.config.can.encoder_msg_rate_ms=10, assign node IDs other "
      "than 0x3f, save_configuration(), and run check_odrive_config --apply.",
      left_seen ? "ok" : "missing", right_seen ? "ok" : "missing");
    feedback_ok_ = false;
    return false;
  }

  feedback_ok_ = true;
  RCLCPP_INFO(
    get_logger(), "Validated cyclic encoder feedback from nodes %u and %u",
    static_cast<unsigned>(left_axis_id_), static_cast<unsigned>(right_axis_id_));
  return true;
}

void ODriveWheelsDriverNode::check_temperature(
    float fet_temp, float motor_temp, int& thermal_level) {
  // Each sensor is compared against its OWN limit (FET vs max_fet_temp,
  // motor vs max_motor_temp). A critical axis stays latched critical until it
  // cools below the warning thresholds — hysteresis that prevents re-arm /
  // shutdown flapping near the limit.
  const bool critical = fet_temp > max_fet_temp_ + critical_temp_offset_ ||
                        motor_temp > max_motor_temp_ + critical_temp_offset_;
  const bool warning = fet_temp > max_fet_temp_ || motor_temp > max_motor_temp_;

  if (critical) {
    thermal_level = 2;
  } else if (warning) {
    if (thermal_level < 2) thermal_level = 1;
  } else {
    thermal_level = 0;
  }

  // Report and act on the worst axis, not the most recently updated one —
  // otherwise a healthy axis's frame would overwrite the hot axis's verdict.
  const int worst = std::max(left_thermal_level_, right_thermal_level_);
  if (worst >= 2) {
    thermal_state_ = "critical";
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
      "CRITICAL: Temperature limit exceeded (FET=%.1f Motor=%.1f), disabling motors",
      static_cast<double>(fet_temp), static_cast<double>(motor_temp));
    stop_motors();
  } else if (worst == 1) {
    thermal_state_ = "warning";
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Temperature warning: FET=%.1f (limit %.1f) Motor=%.1f (limit %.1f)",
      static_cast<double>(fet_temp), max_fet_temp_,
      static_cast<double>(motor_temp), max_motor_temp_);
  } else {
    thermal_state_ = "ok";
  }
}

// ── Event-driven CAN receive ──

void ODriveWheelsDriverNode::start_can_event_loop() {
  if (can_event_thread_.joinable()) {
    return;
  }

  const int can_fd = driver_->native_handle();
  if (can_fd < 0) {
    throw std::runtime_error("Cannot start CAN event loop without an open socket");
  }

  can_stop_fd_ = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
  if (can_stop_fd_ < 0) {
    throw std::runtime_error("Failed to create CAN event-loop stop descriptor");
  }

  can_callback_group_ = this->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  can_guard_condition_ = std::make_shared<rclcpp::GuardCondition>(
    this->get_node_base_interface()->get_context());
  can_waitable_ =
    std::make_shared<GuardConditionCallbackWaitable>(
      can_guard_condition_,
      std::bind(&ODriveWheelsDriverNode::on_can_ready, this));
  this->get_node_waitables_interface()->add_waitable(
    can_waitable_, can_callback_group_);

  can_event_stop_.store(false);
  can_event_error_.store(false);
  can_event_thread_ = std::thread(
    &ODriveWheelsDriverNode::can_event_loop, this);
  RCLCPP_INFO(get_logger(), "SocketCAN receive is event-driven (no encoder timer)");
}

void ODriveWheelsDriverNode::stop_can_event_loop() {
  can_event_stop_.store(true);
  if (can_stop_fd_ >= 0) {
    const uint64_t value = 1;
    const auto written = ::write(can_stop_fd_, &value, sizeof(value));
    (void)written;
  }

  {
    std::lock_guard<std::mutex> lock(can_event_mutex_);
    can_event_pending_ = false;
  }
  can_event_cv_.notify_all();

  if (can_event_thread_.joinable()) {
    can_event_thread_.join();
  }
  if (can_stop_fd_ >= 0) {
    ::close(can_stop_fd_);
    can_stop_fd_ = -1;
  }
}

void ODriveWheelsDriverNode::can_event_loop() {
  struct pollfd descriptors[2] {};
  descriptors[0].fd = driver_->native_handle();
  descriptors[0].events = POLLIN;
  descriptors[1].fd = can_stop_fd_;
  descriptors[1].events = POLLIN;

  while (!can_event_stop_.load()) {
    const int result = ::poll(descriptors, 2, -1);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      can_event_error_.store(true);
      can_guard_condition_->trigger();
      break;
    }
    if ((descriptors[1].revents & POLLIN) != 0 || can_event_stop_.load()) {
      break;
    }
    if ((descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      can_event_error_.store(true);
      can_guard_condition_->trigger();
      break;
    }
    if ((descriptors[0].revents & POLLIN) == 0) {
      continue;
    }

    std::unique_lock<std::mutex> lock(can_event_mutex_);
    can_event_pending_ = true;
    can_guard_condition_->trigger();
    can_event_cv_.wait(lock, [this]() {
      return !can_event_pending_ || can_event_stop_.load();
    });
  }
}

void ODriveWheelsDriverNode::on_can_ready() {
  if (can_event_error_.exchange(false)) {
    RCLCPP_ERROR(get_logger(), "SocketCAN receive event loop failed");
  }

  read_can_messages();

  {
    std::lock_guard<std::mutex> lock(can_event_mutex_);
    can_event_pending_ = false;
  }
  can_event_cv_.notify_one();
}

// ── Main loop (50 Hz) ──

void ODriveWheelsDriverNode::main_loop() {
  // Retry CAN with exponential backoff
  if (!driver_->is_connected()) {
    auto now = this->now();
    auto elapsed = (now - last_can_retry_).nanoseconds() / 1000000;
    if (elapsed < can_retry_delay_ms_) return;
    last_can_retry_ = now;
    if (!init_driver()) {
      can_retry_delay_ms_ = std::min(can_retry_delay_ms_ * 2, 3000);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "CAN init failed on %s, next retry in %dms",
        can_interface_.c_str(), can_retry_delay_ms_);
      return;
    }
    RCLCPP_INFO(get_logger(), "CAN connection restored on %s", can_interface_.c_str());
    can_retry_delay_ms_ = 100;
  }

  // Feedback watchdog uses encoder freshness per wheel. A SocketCAN fd stays "open"
  // when the interface drops or the ODrive powers off, so socket state alone
  // cannot detect a dead bus — frame freshness can.
  if (feedback_timeout_ms_ > 0) {
    const auto fb_now = std::chrono::steady_clock::now();
    const auto left_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      fb_now - driver_->left().last_encoder_feedback).count();
    const auto right_age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      fb_now - driver_->right().last_encoder_feedback).count();
    const bool stale = left_age_ms > feedback_timeout_ms_ || right_age_ms > feedback_timeout_ms_;
    if (stale && feedback_ok_) {
      feedback_ok_ = false;
      target_left_turns_ = 0.0f;
      target_right_turns_ = 0.0f;
      left_prev_cmd_ = 0.0f;
      right_prev_cmd_ = 0.0f;
      RCLCPP_ERROR(get_logger(),
        "ODrive encoder feedback lost (left %lld ms, right %lld ms > %d ms) — "
        "stopping and idling both axes",
        static_cast<long long>(left_age_ms), static_cast<long long>(right_age_ms),
        feedback_timeout_ms_);
      stop_motors();
    } else if (!stale && !feedback_ok_) {
      feedback_ok_ = true;
      RCLCPP_INFO(get_logger(), "ODrive encoder feedback restored");
    }
  }

  // cmd_vel timeout: stop if no command received recently
  auto elapsed_ms = (this->now() - last_cmd_vel_time_).nanoseconds() / 1000000;
  if (elapsed_ms > cmd_vel_timeout_ms_ && !e_stop_active_) {
    target_left_turns_ = 0.0f;
    target_right_turns_ = 0.0f;
  }

  // Send the latest target at a fixed rate. This keeps the ODrive input fresh
  // even when the teleop publisher is only updating at keyboard polling speed.
  if (!e_stop_active_ && motors_armed()) {
    const float loop_dt = 1.0f / static_cast<float>(publish_rate_hz_);
    const float left_cmd = apply_wheel_shaping(
      target_left_turns_, left_prev_cmd_, left_sign_ * left_scale_, loop_dt);
    const float right_cmd = apply_wheel_shaping(
      target_right_turns_, right_prev_cmd_, right_sign_ * right_scale_, loop_dt);
    const float left_ff =
      (std::abs(left_cmd) > 0.001f) ? std::copysign(stiction_torque_ff_, left_cmd) : 0.0f;
    const float right_ff =
      (std::abs(right_cmd) > 0.001f) ? std::copysign(stiction_torque_ff_, right_cmd) : 0.0f;

    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 500,
      "CAN loop command: target=(%.4f, %.4f) sent=(%.4f, %.4f) torque_ff=(%.4f, %.4f)",
      static_cast<double>(target_left_turns_), static_cast<double>(target_right_turns_),
      static_cast<double>(left_cmd), static_cast<double>(right_cmd),
      static_cast<double>(left_ff), static_cast<double>(right_ff));

    send_velocity_with_ff(left_axis_id_, left_cmd, left_ff);
    send_velocity_with_ff(right_axis_id_, right_cmd, right_ff);
  }
}

// ── Diagnostics request (1 Hz) ──

void ODriveWheelsDriverNode::diagnostics_request_loop() {
  if (!driver_->is_connected()) return;

  driver_->request_diagnostics();
}

// ── Read CAN messages ──

void ODriveWheelsDriverNode::read_can_messages() {
  while (true) {
    const auto result = driver_->poll_one(0);
    if (!result.frame_received) {
      break;
    }
    if (result.invalid_encoder_frame) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "ODrive node %d: NaN/Inf in encoder feedback — ignoring",
        result.invalid_encoder_node_id);
    }
    if (result.encoder_updated) {
      publish_joint_state(result.encoder_node_id);
    }
  }

  const auto& left = driver_->left();
  const auto& right = driver_->right();
  if (left.errors != 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "ODrive node %d errors: 0x%08X", left_axis_id_, left.errors);
  }
  if (right.errors != 0) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "ODrive node %d errors: 0x%08X", right_axis_id_, right.errors);
  }

  check_temperature(
    left.fet_temperature, left.motor_temperature, left_thermal_level_);
  check_temperature(
    right.fet_temperature, right.motor_temperature, right_thermal_level_);
}

// ── Publish one wheel sample per encoder frame ──

void ODriveWheelsDriverNode::publish_joint_state(uint8_t node_id) {
  const AxisFeedback* feedback = nullptr;
  const char* joint_name = nullptr;
  double direction = 1.0;

  if (node_id == left_axis_id_) {
    feedback = &driver_->left();
    joint_name = "left_wheel_joint";
    direction = left_sign_;
  } else if (node_id == right_axis_id_) {
    feedback = &driver_->right();
    joint_name = "right_wheel_joint";
    direction = right_sign_;
  }

  if (feedback == nullptr || !feedback->encoder_valid) {
    return;
  }

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = {joint_name};

  // ODrive reports motor turns. JointState represents wheel radians after
  // direction correction and gearbox reduction.
  msg.position = {
    kinematics::motor_turns_to_wheel_radians(
      feedback->position_turns * direction, gear_ratio_)
  };
  msg.velocity = {
    kinematics::motor_turns_to_wheel_radians(
      feedback->velocity_turns_per_s * direction, gear_ratio_)
  };

  pub_joint_->publish(msg);
}

// ── cmd_vel callback ──

void ODriveWheelsDriverNode::on_cmd_vel(const geometry_msgs::msg::Twist& msg) {
  RCLCPP_DEBUG(get_logger(), "cmd_vel received: linear=%.4f angular=%.4f",
               msg.linear.x, msg.angular.z);

  if (e_stop_active_) {
    RCLCPP_DEBUG(get_logger(), "cmd_vel ignored: e_stop is active");
    return;
  }
  if (!motors_armed()) {
    const auto& left = driver_->left();
    const auto& right = driver_->right();
    RCLCPP_DEBUG(get_logger(),
      "cmd_vel ignored: motors not armed or feedback stale "
      "(left_state=%d right_state=%d feedback_ok=%s)",
      left.state, right.state, feedback_ok_ ? "true" : "false");
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
      "cmd_vel received but motors not armed (left=%d, right=%d)",
      left.state, right.state);
    return;
  }

  last_cmd_vel_time_ = this->now();

  // Differential drive inverse kinematics: m/s → motor turns/s
  auto wheels = kinematics::cmd_vel_to_wheels(msg.linear.x, msg.angular.z,
                                              wheel_radius_, track_width_, gear_ratio_);
  const double max_requested_turns =
    std::max(std::abs(wheels.left), std::abs(wheels.right));
  if (max_requested_turns > 0.0) {
    const float required_vel_limit =
      static_cast<float>(max_requested_turns * 1.05);
    if (!ensure_velocity_limit(required_vel_limit)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "cmd_vel ignored: failed to raise ODrive vel_limit to %.3f turns/s",
        static_cast<double>(required_vel_limit));
      return;
    }
  }
  RCLCPP_DEBUG(get_logger(),
    "cmd_vel IK target: left=%.4f turns/s right=%.4f turns/s "
    "(wheel_radius=%.4f track_width=%.4f gear_ratio=%.2f)",
    wheels.left, wheels.right, wheel_radius_, track_width_, gear_ratio_);

  target_left_turns_ = static_cast<float>(wheels.left);
  target_right_turns_ = static_cast<float>(wheels.right);
  RCLCPP_DEBUG(get_logger(),
    "cmd_vel target stored: left=%.4f right=%.4f turns/s",
    static_cast<double>(target_left_turns_),
    static_cast<double>(target_right_turns_));
}

// ── E-stop callback ──

void ODriveWheelsDriverNode::on_e_stop(const std_msgs::msg::Bool& msg) {
  RCLCPP_DEBUG(get_logger(), "e_stop received: %s", msg.data ? "true" : "false");
  if (msg.data && !e_stop_active_) {
    RCLCPP_WARN(get_logger(), "E-STOP ACTIVATED");
    e_stop_active_ = true;
    target_left_turns_ = 0.0f;
    target_right_turns_ = 0.0f;
    left_prev_cmd_ = 0.0f;
    right_prev_cmd_ = 0.0f;
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
  } else if (!msg.data && e_stop_active_) {
    RCLCPP_INFO(get_logger(), "E-stop released");
    e_stop_active_ = false;
  }
}

// ── Wheel command shaping ──

float ODriveWheelsDriverNode::apply_wheel_shaping(float target, float& prev_cmd, double sign_and_scale,
                                         float dt) {
  // Zero-command bypass: release → exact zero, no accel-limiter creep, no min_effective snap
  if (std::abs(target) < zero_vel_epsilon_) {
    prev_cmd = 0.0f;
    return 0.0f;
  }

  float vel = target * static_cast<float>(sign_and_scale);

  // Asymmetric accel limiter: decel is faster than accel for responsive stopping
  bool decelerating = (std::abs(vel) < std::abs(prev_cmd));
  float max_dv = (decelerating ? max_wheel_decel_ : max_wheel_accel_) * dt;
  vel = prev_cmd + std::clamp(vel - prev_cmd, -max_dv, max_dv);

  // Min effective velocity (stiction compensation) — disabled by default (min_effective_vel=0)
  if (min_effective_vel_ > 0.0f && std::abs(vel) > 0.001f && std::abs(vel) < min_effective_vel_) {
    vel = std::copysign(min_effective_vel_, vel);
  }

  prev_cmd = vel;
  return vel;
}

// ── CAN send helpers ──

void ODriveWheelsDriverNode::send_velocity(uint8_t node_id, float vel_turns_per_s) {
  send_velocity_with_ff(node_id, vel_turns_per_s, 0.0f);
}

void ODriveWheelsDriverNode::send_velocity_with_ff(uint8_t node_id, float vel_turns_per_s, float torque_ff) {
  if (!driver_->is_connected()) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "velocity command skipped: CAN driver is not connected");
    return;
  }
  RCLCPP_DEBUG(get_logger(), "CAN velocity command: node=%u vel=%.4f torque_ff=%.4f",
               static_cast<unsigned>(node_id),
               static_cast<double>(vel_turns_per_s),
               static_cast<double>(torque_ff));
  if (!driver_->set_velocity(node_id, vel_turns_per_s, torque_ff)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "CAN send failed (%d consecutive) on %s",
      driver_->consecutive_send_failures(), can_interface_.c_str());
  }
}

bool ODriveWheelsDriverNode::ensure_velocity_limit(float required_vel_limit) {
  if (required_vel_limit <= 0.0f) {
    return true;
  }

  const float target_limit = std::max(required_vel_limit, vel_limit_);
  if (target_limit <= applied_vel_limit_ + 0.25f) {
    return true;
  }

  RCLCPP_DEBUG(get_logger(),
    "raising ODrive vel_limit for cmd_vel: current=%.3f required=%.3f target=%.3f turns/s",
    static_cast<double>(applied_vel_limit_),
    static_cast<double>(required_vel_limit),
    static_cast<double>(target_limit));

  const bool left_ok = driver_->set_velocity_limit(left_axis_id_, target_limit);
  const bool right_ok = driver_->set_velocity_limit(right_axis_id_, target_limit);
  if (!left_ok || !right_ok) {
    return false;
  }

  applied_vel_limit_ = target_limit;
  return true;
}

void ODriveWheelsDriverNode::send_axis_state(uint8_t node_id, AxisState state) {
  if (!driver_->is_connected()) {
    RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
      "axis-state command skipped: CAN driver is not connected");
    return;
  }
  RCLCPP_DEBUG(get_logger(), "CAN axis-state command: node=%u state=%u",
               static_cast<unsigned>(node_id),
               static_cast<unsigned>(state));
  if (!driver_->set_axis_state(node_id, state)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "CAN send failed (%d consecutive) on %s",
      driver_->consecutive_send_failures(), can_interface_.c_str());
  }
}

void ODriveWheelsDriverNode::stop_motors() {
  RCLCPP_INFO(get_logger(), "Stopping motors...");
  send_velocity(left_axis_id_, 0.0f);
  send_velocity(right_axis_id_, 0.0f);
  send_axis_state(left_axis_id_, AxisState::IDLE);
  send_axis_state(right_axis_id_, AxisState::IDLE);
}

// ── Motor readiness ──

bool ODriveWheelsDriverNode::motors_armed() const {
  // Stale feedback means the axis states are unknown — report disarmed so
  // cmd_vel is not forwarded into a dead bus and the operator sees the fault.
  const auto& left = driver_->left();
  const auto& right = driver_->right();
  return feedback_ok_ &&
         left.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL) &&
         right.state == static_cast<uint8_t>(AxisState::CLOSED_LOOP_CONTROL);
}

void ODriveWheelsDriverNode::publish_motor_state() {
  const auto& left = driver_->left();
  const auto& right = driver_->right();
  std_msgs::msg::String msg;
  char buf[512];
  std::snprintf(buf, sizeof(buf),
    R"({"left_state":%d,"right_state":%d,"left_errors":%u,"right_errors":%u,"armed":%s,)"
    R"("bus_voltage":%.2f,"bus_current":%.2f,)"
    R"("left_fet_temp":%.1f,"left_motor_temp":%.1f,)"
    R"("right_fet_temp":%.1f,"right_motor_temp":%.1f,)"
    R"("feedback_ok":%s,"thermal_state":"%s"})",
    left.state, right.state, left.errors, right.errors,
    motors_armed() ? "true" : "false",
    driver_->bus_voltage(), driver_->bus_current(),
    static_cast<double>(left.fet_temperature), static_cast<double>(left.motor_temperature),
    static_cast<double>(right.fet_temperature), static_cast<double>(right.motor_temperature),
    feedback_ok_ ? "true" : "false",
    thermal_state_.c_str());
  msg.data = buf;
  pub_motor_state_->publish(msg);
}

void ODriveWheelsDriverNode::on_motor_enable(const std_msgs::msg::Bool& msg) {
  RCLCPP_DEBUG(get_logger(), "motor_enable received: %s", msg.data ? "true" : "false");
  if (msg.data) {
    if (thermal_state_ == "critical") {
      RCLCPP_DEBUG(get_logger(), "motor_enable ignored: thermal_state=critical");
      RCLCPP_WARN(get_logger(),
        "motor_enable rejected: thermal state is critical (must cool below warning limits)");
      return;
    }
    RCLCPP_INFO(get_logger(), "Enabling motors → CLOSED_LOOP_CONTROL");
    const bool left_gains_ok = driver_->set_velocity_gains(
      left_axis_id_, vel_gain_, vel_integrator_gain_);
    const bool right_gains_ok = driver_->set_velocity_gains(
      right_axis_id_, vel_gain_, vel_integrator_gain_);
    const bool left_limit_ok = driver_->set_velocity_limit(left_axis_id_, vel_limit_);
    const bool right_limit_ok = driver_->set_velocity_limit(right_axis_id_, vel_limit_);
    const bool left_mode_ok = driver_->set_controller_mode(
      left_axis_id_, control_mode::VELOCITY, input_mode::PASSTHROUGH);
    const bool right_mode_ok = driver_->set_controller_mode(
      right_axis_id_, control_mode::VELOCITY, input_mode::PASSTHROUGH);
    RCLCPP_DEBUG(get_logger(),
      "motor_enable reapply config: gains=(%s,%s) limits=(%s,%s) mode=(%s,%s)",
      left_gains_ok ? "ok" : "fail",
      right_gains_ok ? "ok" : "fail",
      left_limit_ok ? "ok" : "fail",
      right_limit_ok ? "ok" : "fail",
      left_mode_ok ? "ok" : "fail",
      right_mode_ok ? "ok" : "fail");
    if (!left_gains_ok || !right_gains_ok || !left_limit_ok || !right_limit_ok ||
        !left_mode_ok || !right_mode_ok) {
      RCLCPP_WARN(get_logger(),
        "motor_enable: failed to reapply velocity gains, limits, or controller mode");
      return;
    }
    applied_vel_limit_ = vel_limit_;
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
    send_axis_state(left_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
    send_axis_state(right_axis_id_, AxisState::CLOSED_LOOP_CONTROL);
  } else {
    RCLCPP_DEBUG(get_logger(), "motor_enable false: zero velocity and request IDLE");
    RCLCPP_INFO(get_logger(), "Disabling motors → IDLE");
    target_left_turns_ = 0.0f;
    target_right_turns_ = 0.0f;
    left_prev_cmd_ = 0.0f;
    right_prev_cmd_ = 0.0f;
    send_velocity(left_axis_id_, 0.0f);
    send_velocity(right_axis_id_, 0.0f);
    send_axis_state(left_axis_id_, AxisState::IDLE);
    send_axis_state(right_axis_id_, AxisState::IDLE);
  }
}

}  // namespace odrive_wheels_driver

// NOTA: FALTA AGREGAR ALERTA PARA EL BLACK-BOX
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<odrive_wheels_driver::ODriveWheelsDriverNode>();
  if (!node->is_can_connected()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to connect to the configured SocketCAN interface");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "SocketCAN connection validated");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
