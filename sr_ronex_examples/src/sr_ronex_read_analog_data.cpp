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
 * @file   sr_ronex_read_analog_data.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Read analog data from ronexes.
 **/

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sstream>

#include "sr_ronex_msgs/GeneralIOState.h"
#include "sr_ronex_msgs/PWM.h"
#include "sr_ronex_utilities/sr_ronex_utilities.hpp"

//-------------------------------------------------------------------------------

/**
 * GeneralIOState.msg
 *
 * bool[] digital
 * uint16[] analogue
 * uint16 pwm_clock_divider
 * bool[] input_mode
 **/
void generalIOState_callback(const sr_ronex_msgs::GeneralIOState::ConstPtr& msg)
{
  const std::vector<short unsigned int> &analogue = msg->analogue;
  const unsigned len = analogue.size();
  for (unsigned k = 0; k < len; k++) {
    ROS_INFO( "analogue[%d] = %d ", k, analogue[k] );
  }
}

//-------------------------------------------------------------------------------

/**
  * Find all ronexes listed in the parameter server.
  * Note that the help method ronex::get_ronex_param_id checks 
  * the ronexes already present on the parameter server and returns 
  * an id on which the given ronex is stored on the parameter server.
  **/
void wait_for_ronexes(void)
{
  // Wait until ronexes are loaded.
  ros::Rate loop_rate(10);
  std::string param;
  while ( ros::param::get("/ronex/devices/0/ronex_id", param ) == false )
    {
      ROS_INFO( "Waiting for the ronex to be loaded properly." );
      loop_rate.sleep();
    }
}

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "sr_ronex_read_analog_data");
 
  // Create a handle to this process' node. 
  ros::NodeHandle n;
  
  // Wait until there is at least one ronex.
  wait_for_ronexes();

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic. Messages are passed to a callback function, here
   * called chatterCallback. The second parameter to the subscribe() function is 
   * the size of the message queue.
   **/
  ros::Subscriber sub = n.subscribe( "/ronex/general_io/1/state", 
				     1000,
				     generalIOState_callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   **/
  ros::spin();
  
  return 0;
}

//-------------------------------------------------------------------------------
