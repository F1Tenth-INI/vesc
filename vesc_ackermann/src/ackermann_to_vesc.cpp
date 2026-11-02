// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <cmath>
#include <sstream>
#include <string>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

using ackermann_msgs::msg::AckermannDriveStamped;
using std::placeholders::_1;
using std_msgs::msg::Float64;

namespace
{
double applySlewRateLimit(
  const double target,
  const double previous,
  const double dt_s,
  const double rate_limit_per_s)
{
  if (rate_limit_per_s <= 0.0 || dt_s <= 0.0) {
    return target;
  }
  const double max_step = rate_limit_per_s * dt_s;
  const double delta = target - previous;
  if (delta > max_step) {
    return previous + max_step;
  }
  if (delta < -max_step) {
    return previous - max_step;
  }
  return target;
}
}  // namespace

AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions & options)
: Node("ackermann_to_vesc_node", options)
{
  // get conversion parameters
  speed_to_erpm_gain_ = declare_parameter<double>("speed_to_erpm_gain");
  speed_to_erpm_offset_ = declare_parameter<double>("speed_to_erpm_offset");
  steering_to_servo_gain_ =
    declare_parameter<double>("steering_angle_to_servo_gain");
  steering_to_servo_offset_ =
    declare_parameter<double>("steering_angle_to_servo_offset");
  use_accel_to_current_when_speed_zero_ =
    declare_parameter<bool>("use_accel_to_current_when_speed_zero", true);
  speed_epsilon_ = declare_parameter<double>("speed_epsilon", 1e-3);
  accel_to_current_gain_ = declare_parameter<double>("acceleration_to_current_gain", 0.0);
  decel_to_current_gain_ = declare_parameter<double>("deceleration_to_current_gain", 0.0);
  velocity_to_current_gain_ = declare_parameter<double>("velocity_to_current_gain", 0.0);
  max_accel_current_ = declare_parameter<double>("max_accel_current", 60.0);
  max_brake_current_ = declare_parameter<double>("max_brake_current", 50.0);
  current_slew_rate_a_per_s_ = declare_parameter<double>("current_slew_rate_a_per_s", 0.0);
  speed_slew_rate_erpm_per_s_ = declare_parameter<double>("speed_slew_rate_erpm_per_s", 0.0);
  current_speed_mps_ = 0.0;
  last_current_cmd_ = 0.0;
  last_speed_cmd_ = 0.0;
  has_last_current_cmd_ = false;
  has_last_speed_cmd_ = false;
  last_current_cmd_time_ = now();
  last_speed_cmd_time_ = now();

  // create publishers to vesc electric-RPM (speed) and servo commands
  erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
  current_pub_ = create_publisher<Float64>("commands/motor/current", 10);
  servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

  // subscribe to ackermann topic
  ackermann_sub_ = create_subscription<AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  odom_sub_ = create_subscription<Odometry>(
    "odom", 10, std::bind(&AckermannToVesc::odomCallback, this, _1));
}

void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
{
  // calc steering angle (servo)
  Float64 servo_msg;
  servo_msg.data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  const bool use_accel_mode =
    use_accel_to_current_when_speed_zero_ &&
    std::fabs(cmd->drive.speed) <= speed_epsilon_;

  if (use_accel_mode) {
    const double accel_cmd = cmd->drive.acceleration;
    const double accel_gain = accel_cmd >= 0.0 ? accel_to_current_gain_ : decel_to_current_gain_;
    // A command of exactly zero (dead mode / safety stop) must not drive the motor:
    // sign 0 removes the velocity feed-forward so the commanded current is 0 (coast).
    const double accel_sign = accel_cmd > 0.0 ? 1.0 : (accel_cmd < 0.0 ? -1.0 : 0.0);

    // Map signed longitudinal acceleration to signed motor current.
    // The velocity term is aligned with commanded acceleration direction so
    // braking from forward motion and reverse acceleration both work.
    const double velocity_term = accel_sign * velocity_to_current_gain_ * std::fabs(current_speed_mps_);
    double current_cmd = accel_gain * accel_cmd + velocity_term;
    if (current_cmd > max_accel_current_) {
      current_cmd = max_accel_current_;
    }
    if (current_cmd < -max_brake_current_) {
      current_cmd = -max_brake_current_;
    }
    const auto now_time = now();
    if (has_last_current_cmd_) {
      const double dt_s = (now_time - last_current_cmd_time_).seconds();
      current_cmd = applySlewRateLimit(
        current_cmd, last_current_cmd_, dt_s, current_slew_rate_a_per_s_);
    }
    last_current_cmd_ = current_cmd;
    has_last_current_cmd_ = true;
    last_current_cmd_time_ = now_time;

    Float64 current_msg;
    current_msg.data = current_cmd;

    if (rclcpp::ok()) {
      current_pub_->publish(current_msg);
      servo_pub_->publish(servo_msg);
    }
    return;
  }

  // Speed mode (legacy behavior): use drive.speed directly
  Float64 erpm_msg;
  erpm_msg.data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;
  const auto now_time = now();
  if (has_last_speed_cmd_) {
    const double dt_s = (now_time - last_speed_cmd_time_).seconds();
    erpm_msg.data = applySlewRateLimit(
      erpm_msg.data, last_speed_cmd_, dt_s, speed_slew_rate_erpm_per_s_);
  }
  last_speed_cmd_ = erpm_msg.data;
  has_last_speed_cmd_ = true;
  last_speed_cmd_time_ = now_time;

  if (rclcpp::ok()) {
    erpm_pub_->publish(erpm_msg);
    servo_pub_->publish(servo_msg);
  }
}

void AckermannToVesc::odomCallback(const Odometry::SharedPtr odom)
{
  current_speed_mps_ = odom->twist.twist.linear.x;
}

}  // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
