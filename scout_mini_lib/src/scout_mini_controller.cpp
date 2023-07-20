/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "scout_mini_lib/scout_mini_controller.h"

ScoutMiniController::ScoutMiniController(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, RobotState& robot_state,
                                         MotorState& motor_state, DriverState& driver_state, LightState& light_state)
  : nh_(nh), nh_priv_(nh_priv), can_device_("can0"), wheel_radius_(0.0875), wheel_separation_(0.490)
{
  nh_priv_.getParam("can_device", can_device_);

  can_driver_ = make_shared<can::ThreadedSocketCANInterface>();
  can_bridge_ = make_shared<CANBridge>(&nh_, &nh_priv_, can_driver_, robot_state, motor_state, driver_state,
                                       light_state, wheel_radius_, wheel_separation_);
}

bool ScoutMiniController::init()
{
  if (!can_driver_->init(can_device_, 0, XmlRpcSettings::create(nh_priv_)))
  {
    ROS_FATAL("Failed to initialize can_device at %s", can_device_.c_str());
    return false;
  }
  else
    ROS_INFO("Successfully connected to %s.", can_device_.c_str());

  can_bridge_->setup(nh_priv_);
  this_thread::sleep_for(std::chrono::milliseconds(1000));

  motion_.id = 0x111;
  motion_.dlc = 0x08;

  light_.id = 0x121;
  light_.dlc = 0x08;

  for (size_t i = 0; i < 4; i++)
  {
    motion_.data[i] = 0x00;
    light_.data[i] = 0x00;
  }
  light_.data[0] = 0x01;

  can::Frame enable = can::toframe(string("421#0100000000000000"));
  can_driver_->send(enable);

  return true;
}

void ScoutMiniController::sendMotorCommand(const MotorCommand& cmd)
{
  const double left_vel = (cmd.command[0] + cmd.command[2]) * 0.5 * wheel_radius_;
  const double right_vel = (cmd.command[1] + cmd.command[3]) * 0.5 * wheel_radius_;

  const double linear = (left_vel + right_vel) * 0.5;
  const double angular = (right_vel - left_vel) / wheel_separation_;

  stringstream ss_linear;
  ss_linear << hex << static_cast<int16_t>(linear * 1000.0);
  uint16_t cmd_linear = static_cast<uint16_t>(stoi(ss_linear.str(), nullptr, 16));

  stringstream ss_angular;
  ss_angular << hex << static_cast<int16_t>(angular * 1000.0);
  uint16_t cmd_angular = static_cast<uint16_t>(stoi(ss_angular.str(), nullptr, 16));

  motion_.data[0] = cmd_linear / 0x100;
  motion_.data[1] = cmd_linear % 0x100;
  motion_.data[2] = cmd_angular / 0x100;
  motion_.data[3] = cmd_angular % 0x100;

  can_driver_->send(motion_);
}

void ScoutMiniController::sendLightCommand(const LightCommand& cmd)
{
  light_.data[1] = cmd.mode;
  light_.data[2] = cmd.custom_brightness;

  can_driver_->send(light_);
}

void ScoutMiniController::clearFailure()
{
  can::Frame clear = can::toframe(string("441#0000000000000000"));
  can_driver_->send(clear);
}