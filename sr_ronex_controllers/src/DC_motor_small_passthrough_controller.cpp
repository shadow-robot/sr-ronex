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
 * @file   DC_motor_small_passthrough_controller.cpp
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief Controller for the RoNeX DC_MOTOR_SMALL module.
 **/

#include <sr_ronex_controllers/DC_motor_small_passthrough_controller.h>
#include "sr_ronex_msgs/MotorPacketCommand.h"
#include "pluginlib/class_list_macros.h"
#include <string>

PLUGINLIB_EXPORT_CLASS(ronex::DCMotorSmallPassthroughController, controller_interface::ControllerBase)

namespace ronex
{
DCMotorSmallPassthroughController::DCMotorSmallPassthroughController()
:loop_count_(0)
{
}

bool DCMotorSmallPassthroughController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;

  std::string ronex_id;
  if (!node_.getParam("ronex_id", ronex_id))
  {
    ROS_ERROR("No RoNeX ID given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  // get the path from the parameters
  std::string path;
  int parameter_id = get_ronex_param_id(ronex_id);
  {
    if ( parameter_id == -1 )
    {
      ROS_ERROR_STREAM("Could not find the RoNeX id in the parameter server: " << ronex_id <<
                               " not loading the controller.");
      return false;
    }
    else
    {
      std::stringstream ss;
      ss << "/ronex/devices/" << parameter_id << "/path";
      if ( !ros::param::get(ss.str(), path) )
      {
        ROS_ERROR_STREAM("Couldn't read the parameter " << ss.str() <<
                                 " from the parameter server. Not loading the controller.");
        return false;
      }
    }
  }

  dc_motor_small_ = static_cast<ronex::DCMotor*>(robot->getCustomHW(path));
  if ( dc_motor_small_ == NULL)
  {
    ROS_ERROR_STREAM("Could not find RoNeX module: " << ronex_id << " not loading the controller");
    return false;
  }

  // init the subscribers
  std::stringstream sub_topic;
  for ( size_t i = 0; i < dc_motor_small_->command_.digital_.size(); ++i)
  {
    sub_topic.str("");
    sub_topic << path << "/command/digital/" << i;
    digital_subscribers_.push_back(
            node_.subscribe<std_msgs::Bool>(sub_topic.str(), 1,
                                            boost::bind(&DCMotorSmallPassthroughController::digital_commands_cb,
                                                        this, _1,  i)));
  }

  for ( size_t i = 0; i < dc_motor_small_->command_.motor_packet_command_.size(); ++i)
  {
    sub_topic.str("");
    sub_topic << path << "/command/motor_packet_command/" << i;
    motor_command_subscribers_.push_back(
                node_.subscribe<sr_ronex_msgs::MotorPacketCommand>(sub_topic.str(), 1,
                                                boost::bind(&DCMotorSmallPassthroughController::motor_packet_cb,
                                                            this, _1,  i)));
  }

  return true;
}

void DCMotorSmallPassthroughController::digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index)
{
  dc_motor_small_->command_.digital_[index] = msg->data;
}

void DCMotorSmallPassthroughController::motor_packet_cb(const sr_ronex_msgs::MotorPacketCommandConstPtr &msg, int index)
{
  dc_motor_small_->command_.motor_packet_command_[index].flags = msg->flags;
  dc_motor_small_->command_.motor_packet_command_[index].on_time = msg->onTime;
  dc_motor_small_->command_.motor_packet_command_[index].period = msg->period;
}
void DCMotorSmallPassthroughController::stopping(const ros::Time& time)
{
  for (size_t index = 0; index < dc_motor_small_->command_.motor_packet_command_.size(); ++index)
  {
    ROS_INFO_STREAM("Stopping DC motor");
    dc_motor_small_->command_.motor_packet_command_[index].flags = 0;
    dc_motor_small_->command_.motor_packet_command_[index].on_time = 0;
    dc_motor_small_->command_.motor_packet_command_[index].period = 0;
  }
}
}  // namespace ronex
