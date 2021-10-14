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

#ifndef SCOUT_MINI_BASE__SCOUT_MINI_BASE_H_
#define SCOUT_MINI_BASE__SCOUT_MINI_BASE_H_

#include "ros/ros.h"
#include "controller_manager/controller_manager.h"
#include "realtime_tools/realtime_publisher.h"

#include "scout_mini_lib/scout_mini_controller.h"
#include "scout_mini_lib/scout_mini_hardware.h"
//#include "scout_mini_lib/scout_mini_diagnostics.h"

#include "std_msgs/Bool.h"
#include "scout_mini_msgs/RobotState.h"
#include "scout_mini_msgs/MotorState.h"
#include "scout_mini_msgs/DriverState.h"
#include "scout_mini_msgs/LightState.h"
#include "scout_mini_msgs/LightCommand.h"
#include "std_srvs/Trigger.h"

using namespace std;

class ScoutMiniBase
{
public:
  ScoutMiniBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

  virtual ~ScoutMiniBase() = default;

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief light command callback
   * \param msg Command message
   */
  void LightCallback(const scout_mini_msgs::LightCommand& msg);

  /**
   * \brief Clear the failure message
   */
  bool clearFailure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp);

  /**
   * \brief Publish robot state
   */
  void publishRobotState();

  /**
   * \brief Publish motor state
   */
  void publishMotorState();

  /**
   * \brief Publish driver state
   */
  void publishDriverState();

  /**
   * \brief Publish light feedback data
   */
  void publishLightState();

  /**
   * \brief Loop for publishing
   */
  void publishLoop();

  /**
   * \brief Loop for controller manager
   */
  void controlLoop();

  /// Communication system with robot
  shared_ptr<ScoutMiniController> robot_;

  /// Hardware interface for robot
  shared_ptr<ScoutMiniHardware> hw_;

  /// Controller manager for the infrastructure to interact with controllers
  shared_ptr<controller_manager::ControllerManager> cm_;

  /// Diagnostics system to collect information from hardware drivers and robot
  // shared_ptr<ScoutMiniDiagnostics> diagnostics_;

private:
  /// ROS parameters
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;
  ros::Subscriber sub_light_cmd_;

  /// rostopic publisher
  realtime_tools::RealtimePublisher<scout_mini_msgs::RobotState> rp_robot_state_;
  realtime_tools::RealtimePublisher<scout_mini_msgs::MotorState> rp_motor_state_;
  realtime_tools::RealtimePublisher<scout_mini_msgs::DriverState> rp_driver_state_;
  realtime_tools::RealtimePublisher<scout_mini_msgs::LightState> rp_light_state_;

  ros::ServiceServer clear_failure_;

  /// Robot name
  string robot_name_;

  /// Joint name
  vector<string> joint_ = { "front_left_wheel_joint", "front_right_wheel_joint", "rear_left_wheel_joint",
                            "rear_right_wheel_joint" };

  /// Control frequency
  double control_frequency_;

  /// Motor Command
  MotorCommand motor_cmd_;

  /// Feedback related
  RobotState robot_state_;
  MotorState motor_state_;
  DriverState driver_state_;
  LightState light_state_;
};

#endif  // SCOUT_MINI_BASE__SCOUT_MINI_BASE_H_
