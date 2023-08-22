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

#ifndef SCOUT_MINI_LIB__SCOUT_MINI_CONTROLLER_H_
#define SCOUT_MINI_LIB__SCOUT_MINI_CONTROLLER_H_

#include <string>
#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include "scout_mini_lib/scout_mini_can_bridge.h"

using namespace std;

class ScoutMiniController
{
public:
  ScoutMiniController(ros::NodeHandle& nh, ros::NodeHandle& nh_priv, RobotState& robot_state, MotorState& motor_state,
                      DriverState& driver_state, LightState& light_state);

  virtual ~ScoutMiniController() = default;

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Send the velocity command
   * \param cmd Velocity command message for each motors (rad/s)
   */
  void sendMotorCommand(const MotorCommand& cmd);

  /**
   * \brief Send the light command
   * \param cmd Light command message
   */
  void sendLightCommand(const LightCommand& cmd);

  /**
   * \brief Clearing failure protection
   */
  void clearFailure();

  /// CAN communication
  shared_ptr<can::ThreadedSocketCANInterface> can_driver_;
  shared_ptr<CANBridge> can_bridge_;

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  /// SocketCAN parameters
  string can_device_;
  can::Frame motion_, light_;

  /// Robot parameters
  double wheel_radius_, wheel_separation_;
};

#endif  // SCOUT_MINI_LIB__SCOUT_MINI_CONTROLLER_H_