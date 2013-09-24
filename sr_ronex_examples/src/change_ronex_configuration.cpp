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
 * @file   change_ronex_configuration.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  Changes configuration parameters of a running ronex module.
 **/

//-------------------------------------------------------------------------------
#include <ros/ros.h>
#include <ros/console.h>
#include <string>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

//-------------------------------------------------------------------------------

/**
 * This class demonstrate how to change the configuration parameters of a running ronex module.
 **/
class ChangeRonexConfigurationExample
{
public:

  ChangeRonexConfigurationExample()
  {
  }

  ~ChangeRonexConfigurationExample()
  {
  }

  /**
   * Configuration of the ronex module is done by calling the /ronex/general_io/X/set_parameters service
   * where X is the ronex_id of the module to be configured.
   **/
  void configureRonex(std::string path)
  {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
    dynamic_reconfigure::BoolParameter bool_param;
    dynamic_reconfigure::IntParameter int_param;
    dynamic_reconfigure::Config conf;

    bool_param.name = "input_mode_0";
    bool_param.value = false;
    conf.bools.push_back(bool_param);

    bool_param.name = "input_mode_1";
    bool_param.value = false;
    conf.bools.push_back(bool_param);

    int_param.name = "pwm_period_0";
    int_param.value = 200;
    conf.ints.push_back(int_param);

    int_param.name = "pwm_clock_divider";
    int_param.value = 3000;
    conf.ints.push_back(int_param);

    srv_req.config = conf;

    ros::service::call(path + "/set_parameters", srv_req, srv_resp);
  }

};

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "change_ronex_configuration_cpp");

  // This class demonstrate how to change the configuration parameters of a running ronex module.
  ChangeRonexConfigurationExample example;

  //Define the ronex id of the module to be configured
  std::string ronex_id = "test_ronex";
  std::string ronex_path = "/ronex/general_io/" + ronex_id + "/";

  // Call the actual method to configure the ronex
  example.configureRonex(ronex_path);

  return 0;
}

//-------------------------------------------------------------------------------
