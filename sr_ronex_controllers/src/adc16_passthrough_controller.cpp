/*
 * Copyright (c) 2013, Shadow Robot Company, All rights reserved.
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
 * @file   adc16_passthrough_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A passthrough for the ADC RoNeX module: simply sets the
 *         different pins to the value you want directly.
 **/

#include "sr_ronex_controllers/adc16_passthrough_controller.hpp"
#include "pluginlib/class_list_macros.h"
#include <string>

PLUGINLIB_EXPORT_CLASS(ronex::ADC16PassthroughController, controller_interface::ControllerBase)

namespace ronex
{
ADC16PassthroughController::ADC16PassthroughController()
  : loop_count_(0)
{}

bool ADC16PassthroughController::init(ros_ethercat_model::RobotStateInterface* robot, ros::NodeHandle &n)
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
    if (parameter_id == -1)
    {
      ROS_ERROR_STREAM("Could not find the RoNeX id in the parameter server: " << ronex_id <<
                               " not loading the controller.");
      return false;
    }
    else
    {
      std::stringstream ss;
      ss << "/ronex/devices/" << parameter_id << "/path";
      if (!ros::param::get(ss.str(), path))
      {
        ROS_ERROR_STREAM("Couldn't read the parameter " << ss.str() <<
                                 " from the parameter server. Not loading the controller.");
        return false;
      }
    }
  }

  std::string robot_state_name;
  node_.param<std::string>("robot_state_name", robot_state_name, "unique_robot_hw");

  try
  {
    adc16_ = static_cast<ronex::ADC16*>(robot->getHandle(robot_state_name).getState()->getCustomHW(path));
  }
  catch(const hardware_interface::HardwareInterfaceException& e)
  {
    ROS_ERROR_STREAM("Could not find robot state: " << robot_state_name << " Not loading the controller. " << e.what());
    return false;
  }

  if ( adc16_ == NULL)
  {
    ROS_ERROR_STREAM("Could not find RoNeX module: " << ronex_id << " not loading the controller");
    return false;
  }

  // init the subscribers
  std::stringstream sub_topic;
  for (size_t i = 0; i < adc16_->command_.digital_.size(); ++i)
  {
    sub_topic.str("");
    sub_topic << path << "/command/digital/" << i;
    digital_subscribers_.push_back(
            node_.subscribe<std_msgs::Bool>(sub_topic.str(), 1,
                                            boost::bind(&ADC16PassthroughController::digital_commands_cb,
                                                        this, _1, i)));
  }

  return true;
}

void ADC16PassthroughController::digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index)
{
  adc16_->command_.digital_[index] = msg->data;
}

}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
