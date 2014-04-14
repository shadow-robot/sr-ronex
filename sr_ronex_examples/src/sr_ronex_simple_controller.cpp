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
 * @file   sr_ronex_simple_controller.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Access RoNeX data directly from the controller.
 **/

#include "sr_ronex_examples/sr_ronex_simple_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::SrRoNeXSimpleController, controller_interface::ControllerBase)

namespace ronex
{
SrRoNeXSimpleController::SrRoNeXSimpleController()
  : loop_count_(0)
{
}

SrRoNeXSimpleController::~SrRoNeXSimpleController()
{
}

bool SrRoNeXSimpleController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  assert (robot);
  
  std::string path("/ronex/general_io/test_ronex");
  general_io_ = static_cast<ronex::GeneralIO*>( robot->getCustomHW(path) );
  if( general_io_ == NULL)
  {
    ROS_ERROR_STREAM("Could not find RoNeX module (i.e., test_ronex). The controller is not loaded.");
    return false;
  }
  
  return true;
}

void SrRoNeXSimpleController::starting()
{
  // Do nothing.
}

void SrRoNeXSimpleController::update(const ros::Time&, const ros::Duration&)
{
  double position = general_io_->state_.analogue_[0];
  if (loop_count_++ % 100 == 0)
  {
    ROS_INFO_STREAM( "Position = " << position );
    loop_count_ = 0;
  }
}

void SrRoNeXSimpleController::stopping() 
{
  // Do nothing.
} 

}

