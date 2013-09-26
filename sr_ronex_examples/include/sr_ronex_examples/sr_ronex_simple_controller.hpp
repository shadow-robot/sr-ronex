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
 * @file   sr_ronex_simple_controller.hpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  A passthrough for the General IO RoNeX module: simply sets 
 *         the different pins to the value you want directly.
 **/

#ifndef _SR_RONEX_SIMPLE_CONTROLLER_
#define _SR_RONEX_SIMPLE_CONTROLLER_

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

#include <std_msgs/Bool.h>
#include <sr_ronex_msgs/PWM.h>

namespace ronex
{
  class SrRoNeXSimpleController 
    : public pr2_controller_interface::Controller
  {
  public:
    SrRoNeXSimpleController();
    virtual ~SrRoNeXSimpleController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    virtual void stopping();

    // void digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index);

    // void pwm_commands_cb(const sr_ronex_msgs::PWMConstPtr& msg, int index);

  private:
    ros::NodeHandle node_;

    int loop_count_;

    ronex::GeneralIO* general_io_;

    ///send commands to the RoNeX's digital I/O
    // std::vector<ros::Subscriber> digital_subscribers_;
    ///send PWM commands to the RoNeX's
    // std::vector<ros::Subscriber> pwm_subscribers_;
  };
}

#endif /* _ACCESS_RONEX_DATA_HPP_ */
