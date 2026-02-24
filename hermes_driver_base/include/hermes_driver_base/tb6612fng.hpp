#ifndef HERMES_DRIVER_BASE__TB6612FNG_HPP_
#define HERMES_DRIVER_BASE__TB6612FNG_HPP_

#include <atomic>
#include <cstdint>
#include <thread>

#include <gpiod.h>

#include "rclcpp/rclcpp.hpp"

namespace hermes_driver
{

/// Pin configuration for a single motor channel on the TB6612FNG.
struct MotorPins
{
  int pwm;   // PWM pin (speed control)
  int in1;   // Direction input 1
  int in2;   // Direction input 2
};

/// Pin configuration for the entire TB6612FNG breakout.
struct TB6612Pins
{
  MotorPins motor_a;  // Left motor  (AIN1, AIN2, PWMA)
  MotorPins motor_b;  // Right motor (BIN1, BIN2, PWMB)
  int stby;           // Standby pin (HIGH = enabled)
};

/// Software PWM state for a single output line.
struct PwmState
{
  gpiod_line * line{nullptr};
  std::atomic<int> duty{0};      // duty cycle 0–100
  std::thread thread;
  std::atomic<bool> running{false};
};

/// Low-level driver for the TB6612FNG dual H-bridge motor driver.
///
/// Responsibilities:
///   - Initialise GPIO pins (via libgpiod)
///   - Accept a speed value [-1.0, 1.0] per motor and translate to
///     PWM duty-cycle + direction pin states
///   - Enable / disable the driver via the STBY pin
class TB6612FNG
{
public:
  /// @param pins        GPIO pin mapping
  /// @param pwm_freq    PWM frequency in Hz (typical: 1000–20000)
  /// @param logger      ROS 2 logger used to report GPIO errors
  explicit TB6612FNG(
    const TB6612Pins & pins,
    int pwm_freq = 1000,
    rclcpp::Logger logger = rclcpp::get_logger("TB6612FNG"));
  ~TB6612FNG();

  // Prevent copies (owns GPIO chip and line handles)
  TB6612FNG(const TB6612FNG &) = delete;
  TB6612FNG & operator=(const TB6612FNG &) = delete;

  /// Initialise the GPIO chip and claim all pins. Returns true on success.
  bool init();

  /// Set motor A speed.  @param speed  [-1.0 .. 1.0]  (negative = reverse)
  void set_motor_a(double speed);

  /// Set motor B speed.  @param speed  [-1.0 .. 1.0]  (negative = reverse)
  void set_motor_b(double speed);

  /// Convenience: set both motors at once.
  void set_motors(double speed_a, double speed_b);

  /// Pull STBY HIGH (enable) or LOW (coast to stop).
  void set_standby(bool enable);

  /// Coast both motors to stop and release GPIO resources.
  void shutdown();

private:
  void set_motor(
    const MotorPins & mp, double speed,
    PwmState & pwm_state, gpiod_line * in1, gpiod_line * in2);
  void start_pwm(PwmState & pwm_state);
  void stop_pwm(PwmState & pwm_state);

  TB6612Pins pins_;
  int pwm_freq_;
  rclcpp::Logger logger_;

  gpiod_chip * chip_{nullptr};

  // Direction / standby lines
  gpiod_line * in1_a_{nullptr};
  gpiod_line * in2_a_{nullptr};
  gpiod_line * in1_b_{nullptr};
  gpiod_line * in2_b_{nullptr};
  gpiod_line * stby_{nullptr};

  // Software PWM channels
  PwmState pwm_a_;
  PwmState pwm_b_;

  bool initialised_{false};
};

}  // namespace hermes_driver

#endif  // HERMES_DRIVER_BASE__TB6612FNG_HPP_