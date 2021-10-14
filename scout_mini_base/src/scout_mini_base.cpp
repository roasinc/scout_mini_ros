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

#include "scout_mini_base/scout_mini_base.h"

ScoutMiniBase::ScoutMiniBase(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : nh_(nh), nh_priv_(nh_priv), robot_name_("scout_mini"), control_frequency_(50.0)
{
  nh_priv_.getParam("robot_name", robot_name_);
  nh_priv_.getParam("control_frequency", control_frequency_);

  robot_ = make_shared<ScoutMiniController>(nh_, nh_priv_, robot_state_, motor_state_, driver_state_, light_state_);
  hw_ = make_shared<ScoutMiniHardware>(joint_);
  cm_ = make_shared<controller_manager::ControllerManager>(hw_.get(), nh_);
  // diagnostics_ = make_shared<RoasDiagnostics>(robot_, feedback_);

  sub_light_cmd_ = nh_.subscribe(robot_name_ + "/light/command", 10, &ScoutMiniBase::LightCallback, this);
  clear_failure_ = nh_.advertiseService(robot_name_ + "/clear_failure", &ScoutMiniBase::clearFailure, this);
}

bool ScoutMiniBase::init()
{
  rp_robot_state_.init(nh_, robot_name_ + "/robot_state", 1);
  rp_motor_state_.init(nh_, robot_name_ + "/motor_state", 1);
  rp_driver_state_.init(nh_, robot_name_ + "/driver_state", 1);
  rp_light_state_.init(nh_, robot_name_ + "/light_state", 1);

  rp_robot_state_.msg_.robot = robot_name_;
  for (size_t i = 0; i < joint_.size(); i++)
    rp_motor_state_.msg_.name[i] = joint_[i];

  return robot_->init();
}

void ScoutMiniBase::LightCallback(const scout_mini_msgs::LightCommand& msg)
{
  LightCommand cmd;
  cmd.mode = msg.mode;
  cmd.custom_brightness = msg.custom_brightness;
  robot_->sendLightCommand(cmd);
}

bool ScoutMiniBase::clearFailure(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
  robot_->clearFailure();
  resp.success = true;
  return true;
}

void ScoutMiniBase::publishRobotState()
{
  if (rp_robot_state_.trylock())
  {
    rp_robot_state_.msg_.header.stamp = ros::Time::now();
    rp_robot_state_.msg_.normal_state = robot_state_.normal_state;
    rp_robot_state_.msg_.control_mode = robot_state_.control_mode;
    rp_robot_state_.msg_.battery_voltage = robot_state_.battery_voltage;
    rp_robot_state_.msg_.fault_state.battery_under_voltage_failure =
        robot_state_.fault_state.battery_under_voltage_failure;
    rp_robot_state_.msg_.fault_state.battery_under_voltage_alarm = robot_state_.fault_state.battery_under_voltage_alarm;
    rp_robot_state_.msg_.fault_state.loss_remote_control = robot_state_.fault_state.loss_remote_control;
    rp_robot_state_.unlockAndPublish();
  }
}

void ScoutMiniBase::publishMotorState()
{
  if (rp_motor_state_.trylock())
  {
    rp_motor_state_.msg_.header.stamp = ros::Time::now();

    for (size_t i = 0; i < joint_.size(); i++)
    {
      rp_motor_state_.msg_.position[i] = motor_state_.position[i];
      rp_motor_state_.msg_.velocity[i] = motor_state_.velocity[i];
      rp_motor_state_.msg_.current[i] = motor_state_.current[i];
      rp_motor_state_.msg_.temperature[i] = motor_state_.temperature[i];
    }

    rp_motor_state_.unlockAndPublish();
  }
}

void ScoutMiniBase::publishDriverState()
{
  if (rp_driver_state_.trylock())
  {
    rp_driver_state_.msg_.header.stamp = ros::Time::now();

    for (size_t i = 0; i < joint_.size(); i++)
    {
      rp_driver_state_.msg_.driver_voltage[i] = driver_state_.driver_voltage[i];
      rp_driver_state_.msg_.driver_temperature[i] = driver_state_.driver_temperature[i];
      rp_driver_state_.msg_.communication_failure[i] = driver_state_.communication_failure[i];
      rp_driver_state_.msg_.low_supply_voltage[i] = driver_state_.low_supply_voltage[i];
      rp_driver_state_.msg_.motor_over_temperature[i] = driver_state_.motor_over_temperature[i];
      rp_driver_state_.msg_.driver_over_current[i] = driver_state_.driver_over_current[i];
      rp_driver_state_.msg_.driver_over_temperature[i] = driver_state_.driver_over_temperature[i];
    }

    rp_driver_state_.unlockAndPublish();
  }
}

void ScoutMiniBase::publishLightState()
{
  if (rp_light_state_.trylock())
  {
    rp_light_state_.msg_.header.stamp = ros::Time::now();
    rp_light_state_.msg_.control_enable = light_state_.control_enable;
    rp_light_state_.msg_.control_enable = light_state_.control_enable;
    rp_light_state_.msg_.mode = light_state_.mode;
    rp_light_state_.msg_.brightness = light_state_.brightness;

    rp_light_state_.unlockAndPublish();
  }
}

void ScoutMiniBase::publishLoop()
{
  while (ros::ok())
  {
    publishRobotState();
    publishMotorState();
    publishDriverState();
    publishLightState();
    // diagnostics_->updateDiagnosticsMessage();

    ros::Rate(control_frequency_).sleep();
  }
}

void ScoutMiniBase::controlLoop()
{
  chrono::steady_clock::time_point last_time = chrono::steady_clock::now();

  while (ros::ok())
  {
    chrono::steady_clock::time_point current_time = chrono::steady_clock::now();
    chrono::duration<double> elapsed_time = current_time - last_time;
    ros::Duration elapsed(elapsed_time.count());
    last_time = current_time;

    hw_->receiveMotorState(motor_state_);
    cm_->update(ros::Time::now(), elapsed);
    hw_->writeCommand(motor_cmd_);
    robot_->sendMotorCommand(motor_cmd_);

    ros::Rate(control_frequency_).sleep();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS node
  ros::init(argc, argv, "scout_mini_base_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  auto scout_mini = make_shared<ScoutMiniBase>(nh, nh_priv);

  if (scout_mini->init())
  {
    // Create thread for controller manager loop
    thread control([&scout_mini]() -> void { scout_mini->controlLoop(); });

    // Create thread for publishing the feedback data
    thread publish([&scout_mini]() -> void { scout_mini->publishLoop(); });

    ros::spin();
  }

  return EXIT_FAILURE;
}