/*
 * Copyright (c) 2016, Shadow Robot Company, All rights reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library.
 */

/**
 * @file   DC_motor_small_passthrough_controller.h
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief Controller for the RoNeX DC_MOTOR_SMALL module.
 **/

#ifndef SR_RONEX_CONTROLLERS_DC_MOTOR_SMALL_PASSTHROUGH_CONTROLLER_H
#define SR_RONEX_CONTROLLERS_DC_MOTOR_SMALL_PASSTHROUGH_CONTROLLER_H

#include <ros/node_handle.h>

#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <sr_ronex_hardware_interface/DC_motor_small_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

#include <std_msgs/Bool.h>
#include "sr_ronex_msgs/MotorPacketStatus.h"
#include "sr_ronex_msgs/MotorPacketCommand.h"
#include <vector>

namespace ronex
{
class DCMotorSmallPassthroughController
    : public controller_interface::Controller<ros_ethercat_model::RobotState>
{
public:
  DCMotorSmallPassthroughController();

  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time&, const ros::Duration&) {}

  virtual void stopping(const ros::Time& time);

  void digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index);

  void motor_packet_cb(const sr_ronex_msgs::MotorPacketCommandConstPtr &msg, int index);

private:
  ros::NodeHandle node_;

  int loop_count_;

  ronex::DCMotor* dc_motor_small_;

  /// send commands to the RoNeX's digital I/O
  std::vector<ros::Subscriber> digital_subscribers_;

  /// send commands to the RoNeX's Motor controllers
  std::vector<ros::Subscriber> motor_command_subscribers_;
};
}  // namespace ronex
#endif /* SR_RONEX_CONTROLLERS_DC_MOTOR_SMALL_PASSTHROUGH_CONTROLLER_H */
