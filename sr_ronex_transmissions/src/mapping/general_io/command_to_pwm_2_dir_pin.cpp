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
* @file command_to_pwm.hpp
* @author Toni Oliver <toni@shadowrobot.com>
* @brief  Contains the data mapping one joint command to a pwm module. It uses 2 opposite direction pins (unlike
* CommandToPWM, that uses a single one)
**/

#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm_2_dir_pin.hpp"
#include <ros_ethercat_model/robot_state.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>

namespace ronex
{
namespace mapping
{
namespace general_io
{
CommandToPWM2PinDir::CommandToPWM2PinDir(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot)
  : CommandToPWM(mapping_el, robot)
{
  // Stop the timer created by the parent class constructor. It would call the wrong try_init_cb_ (wouldn't it?)
  init_timer_.stop();

  const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
  if (!ronex_name)
  {
    ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
    return;
  }

  init_timer_ = nh_.createTimer(ros::Duration(0.01),
                                boost::bind(&CommandToPWM2PinDir::try_init_cb_, this, _1, mapping_el,
                                            robot, ronex_name));
}

bool CommandToPWM2PinDir::try_init_cb_(const ros::TimerEvent&, TiXmlElement* mapping_el,
                                       ros_ethercat_model::RobotState* robot, const char* ronex_name)
{
  if ( !init_(mapping_el, robot, ronex_name) )
  {
    return false;
  }

  ROS_DEBUG_STREAM("RoNeX" << ronex_name << " is initialised now.");
  // stopping timer
  init_timer_.stop();

  is_initialized_ = true;
  return true;
}

bool CommandToPWM2PinDir::init_(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name)
{
  if ( !CommandToPWM::init_(mapping_el, robot, ronex_name))
  {
    return false;
  }

  // read motor direction pin 2 index from urdf
  const char *d_pin = mapping_el ? mapping_el->Attribute("direction_pin_2") : NULL;
  if (!d_pin)
  {
    ROS_ERROR("RonexTransmission transmission did not specify the direction pin 2.");
    return false;
  }
  // convert to size_t
  try
  {
    digital_pin_index_2_ = boost::lexical_cast<size_t>(d_pin);
  }
  catch( boost::bad_lexical_cast const& )
  {
    ROS_ERROR("RonexTransmission: Couldn't parse direction_pin_2 to a size_t.");
    return false;
  }

  return true;
}

bool CommandToPWM2PinDir::check_pins_in_bound_()
{
  // we ignore the first iteration as the array is not yet initialised.
  if ( first_iteration_ )
  {
    pin_out_of_bound_ = true;
    first_iteration_ = false;
    return false;
  }

  pin_out_of_bound_ = !CommandToPWM::check_pins_in_bound_();

  if ( !pin_out_of_bound_ )
  {
    if ( digital_pin_index_2_ > general_io_->command_.digital_.size() )
    {
      // size_t is always >= 0 so no need to check lower bound
      ROS_ERROR_STREAM("Specified direction pin 2 is out of bound: " << digital_pin_index_2_ << " / max = " <<
                               general_io_->command_.digital_.size() << " , not propagating the command to the RoNeX.");
      pin_out_of_bound_ = true;
      return false;
    }
  }

  pin_out_of_bound_ = false;
  return true;
}

void CommandToPWM2PinDir::propagateToRonex(ros_ethercat_model::JointState *js)
{
  if ( !is_initialized_ )
    return;

  if ( check_pins_in_bound_() )
  {
    if ( pwm_pin_index_ == 0 )
      general_io_->command_.pwm_[pwm_module_].on_time_0 =
              static_cast<uint16_t>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) *
                      abs(js->commanded_effort_) ) / 100);
    else
      general_io_->command_.pwm_[pwm_module_].on_time_1 =
              static_cast<uint16_t>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) *
                      abs(js->commanded_effort_) ) / 100);

    // This is assigned by convention: negative effort sets the direction pin to 1.
    general_io_->command_.digital_[digital_pin_index_] = (js->commanded_effort_ < 0.0);
    // The 2nd direction pin is the opposite of the first one
    general_io_->command_.digital_[digital_pin_index_2_] = !general_io_->command_.digital_[digital_pin_index_];
  }
}
}  // namespace general_io
}  // namespace mapping
}  // namespace ronex
/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
