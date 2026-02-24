#include "hermes_driver_base/tb6612fng.hpp"

#include <gpiod.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <thread>

namespace hermes_driver
{

TB6612FNG::TB6612FNG(const TB6612Pins & pins, int pwm_freq, rclcpp::Logger logger)
: pins_(pins), pwm_freq_(pwm_freq), logger_(logger)
{}

TB6612FNG::~TB6612FNG()
{
  shutdown();
}

/// Helper: get and claim a single GPIO line as an output.
/// Returns the line pointer on success, nullptr on failure.
static gpiod_line * claim_output(
  gpiod_chip * chip, int pin, const char * label, rclcpp::Logger logger)
{
  gpiod_line * line = gpiod_chip_get_line(chip, static_cast<unsigned int>(pin));
  if (!line) {
    RCLCPP_ERROR(logger,
      "[TB6612FNG] Failed to get GPIO line for pin %d (%s): %s",
      pin, label, std::strerror(errno));
    return nullptr;
  }
  if (gpiod_line_request_output(line, "hermes_driver", 0) < 0) {
    RCLCPP_ERROR(logger,
      "[TB6612FNG] Failed to claim pin %d (%s) as output: %s",
      pin, label, std::strerror(errno));
    return nullptr;
  }
  return line;
}

bool TB6612FNG::init()
{
  // Open the GPIO chip (4 for rpi5)
  chip_ = gpiod_chip_open_by_number(4);
  if (!chip_) {
    RCLCPP_ERROR(logger_,
      "[TB6612FNG] Failed to open GPIO chip 4: %s", std::strerror(errno));
    return false;
  }

  // Claim direction and standby pins as outputs
  in1_a_ = claim_output(chip_, pins_.motor_a.in1, "AIN1", logger_);
  in2_a_ = claim_output(chip_, pins_.motor_a.in2, "AIN2", logger_);
  in1_b_ = claim_output(chip_, pins_.motor_b.in1, "BIN1", logger_);
  in2_b_ = claim_output(chip_, pins_.motor_b.in2, "BIN2", logger_);
  stby_  = claim_output(chip_, pins_.stby,         "STBY", logger_);

  if (!in1_a_ || !in2_a_ || !in1_b_ || !in2_b_ || !stby_) {
    RCLCPP_ERROR(logger_, "[TB6612FNG] Failed to claim direction/stby pins");
    return false;
  }

  // Claim PWM pins as outputs (driven by software-PWM threads)
  pwm_a_.line = claim_output(chip_, pins_.motor_a.pwm, "PWMA", logger_);
  pwm_b_.line = claim_output(chip_, pins_.motor_b.pwm, "PWMB", logger_);

  if (!pwm_a_.line || !pwm_b_.line) {
    RCLCPP_ERROR(logger_, "[TB6612FNG] Failed to claim PWM pins");
    return false;
  }

  // Start software PWM threads
  start_pwm(pwm_a_);
  start_pwm(pwm_b_);

  // Enable the driver
  if (gpiod_line_set_value(stby_, 1) < 0) {
    RCLCPP_ERROR(logger_,
      "[TB6612FNG] Failed to set STBY HIGH: %s", std::strerror(errno));
    return false;
  }

  initialised_ = true;
  return true;
}

void TB6612FNG::start_pwm(PwmState & pwm_state)
{
  pwm_state.duty.store(0);
  pwm_state.running.store(true);
  int freq = pwm_freq_;
  // NOTE: pwm_state is a member variable; the thread is always joined in
  // stop_pwm() (called from shutdown()) before any members are destroyed,
  // so capturing by reference is safe.
  pwm_state.thread = std::thread(
    [&pwm_state, freq]() {
      constexpr int kUsPerSec = 1000000;
      const int period_us = kUsPerSec / freq;
      bool write_error = false;   // track error state to log transitions only
      while (pwm_state.running.load()) {
        int duty = pwm_state.duty.load();
        int rc = 0;
        if (duty <= 0) {
          rc = gpiod_line_set_value(pwm_state.line, 0);
          std::this_thread::sleep_for(std::chrono::microseconds(period_us));
        } else if (duty >= 100) {
          rc = gpiod_line_set_value(pwm_state.line, 1);
          std::this_thread::sleep_for(std::chrono::microseconds(period_us));
        } else {
          int high_us = period_us * duty / 100;
          int low_us  = period_us - high_us;
          rc  = gpiod_line_set_value(pwm_state.line, 1);
          std::this_thread::sleep_for(std::chrono::microseconds(high_us));
          rc |= gpiod_line_set_value(pwm_state.line, 0);
          std::this_thread::sleep_for(std::chrono::microseconds(low_us));
        }
        // Log only on error-state transitions to avoid flooding at PWM frequency
        if (rc < 0 && !write_error) {
          write_error = true;
          // Use fprintf here since RCLCPP is not safe to call from a plain thread
          // without a node context; the error will surface at the driver level.
          std::fprintf(stderr,
            "[TB6612FNG] PWM line write failed (subsequent failures suppressed)\n");
        } else if (rc == 0) {
          write_error = false;
        }
      }
      // Ensure line is LOW when the thread exits
      gpiod_line_set_value(pwm_state.line, 0);
    });
}

void TB6612FNG::stop_pwm(PwmState & pwm_state)
{
  if (pwm_state.running.load()) {
    pwm_state.duty.store(0);
    pwm_state.running.store(false);
    if (pwm_state.thread.joinable()) {
      pwm_state.thread.join();
    }
  }
}

void TB6612FNG::set_motor(
  const MotorPins & mp, double speed,
  PwmState & pwm_state, gpiod_line * in1, gpiod_line * in2)
{
  if (!initialised_) {return;}

  speed = std::clamp(speed, -1.0, 1.0);

  // Direction logic:
  //   speed > 0  →  IN1 = HIGH, IN2 = LOW   (forward)
  //   speed < 0  →  IN1 = LOW,  IN2 = HIGH  (reverse)
  //   speed == 0 →  IN1 = LOW,  IN2 = LOW   (coast / brake)
  int in1_val = (speed > 0.0) ? 1 : 0;
  int in2_val = (speed < 0.0) ? 1 : 0;

  if (gpiod_line_set_value(in1, in1_val) < 0) {
    RCLCPP_ERROR(logger_,
      "[TB6612FNG] Failed to write IN1 (pin %d): %s", mp.in1, std::strerror(errno));
  }
  if (gpiod_line_set_value(in2, in2_val) < 0) {
    RCLCPP_ERROR(logger_,
      "[TB6612FNG] Failed to write IN2 (pin %d): %s", mp.in2, std::strerror(errno));
  }

  // Update duty cycle for the software PWM thread (0–100)
  pwm_state.duty.store(static_cast<int>(std::abs(speed) * 100.0));
}

void TB6612FNG::set_motor_a(double speed)
{
  set_motor(pins_.motor_a, speed, pwm_a_, in1_a_, in2_a_);
}

void TB6612FNG::set_motor_b(double speed)
{
  set_motor(pins_.motor_b, speed, pwm_b_, in1_b_, in2_b_);
}

// Assumes motor a controls left wheel and motor b controls right wheel
void TB6612FNG::set_motors(double speed_a, double speed_b)
{
  set_motor_a(speed_a);
  set_motor_b(speed_b);
}

void TB6612FNG::set_standby(bool enable)
{
  if (!initialised_) {return;}
  if (gpiod_line_set_value(stby_, enable ? 1 : 0) < 0) {
    RCLCPP_ERROR(logger_,
      "[TB6612FNG] Failed to set STBY %s: %s",
      enable ? "HIGH" : "LOW", std::strerror(errno));
  }
}

void TB6612FNG::shutdown()
{
  if (!initialised_) {return;}

  // Stop both motors and disable the driver
  set_motors(0.0, 0.0);
  set_standby(false);

  // Stop software PWM threads
  stop_pwm(pwm_a_);
  stop_pwm(pwm_b_);

  // Release all GPIO lines, then close the chip
  if (in1_a_) { gpiod_line_release(in1_a_); in1_a_ = nullptr; }
  if (in2_a_) { gpiod_line_release(in2_a_); in2_a_ = nullptr; }
  if (in1_b_) { gpiod_line_release(in1_b_); in1_b_ = nullptr; }
  if (in2_b_) { gpiod_line_release(in2_b_); in2_b_ = nullptr; }
  if (stby_)  { gpiod_line_release(stby_);  stby_  = nullptr; }
  if (pwm_a_.line) { gpiod_line_release(pwm_a_.line); pwm_a_.line = nullptr; }
  if (pwm_b_.line) { gpiod_line_release(pwm_b_.line); pwm_b_.line = nullptr; }

  if (chip_) { gpiod_chip_close(chip_); chip_ = nullptr; }

  initialised_ = false;
}

}  // namespace hermes_driver