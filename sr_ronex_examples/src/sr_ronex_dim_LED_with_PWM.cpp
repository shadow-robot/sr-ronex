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
 * @file   sr_ronex_dim_LED_with_PWM.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Read analog data from ronexes.
 **/

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sstream>

#include "sr_ronex_msgs/PWM.h"
#include "sr_ronex_utilities/sr_ronex_utilities.hpp"

//-------------------------------------------------------------------------------

/**
 * This class demonstrates how to find a ronex listed in the parameter server,
 * and then get the information about its path.
 **/
class SrRonexExample
{
public:
  SrRonexExample() {}
  ~SrRonexExample() {}
  
public:
  /**
   * Find the path of the ronex with the given ronex_id.
   *
   * @param ronex_id Select the ronex.
   * @param path The path of the ronex with the selected ronex.
   * @return True if the ronex is found and the path is set. Otherwise, false.
   **/
  bool get_ronex_path_( const std::string& ronex_id, std::string& path )
  {
    // Wait until ronexes are loaded.
    ros::Rate loop_rate(10);
    std::string param;
    while ( ros::param::get("/ronex/devices/0/ronex_id", param ) == false )
    {
      ROS_INFO( "Waiting for the ronex to be loaded properly." );
      loop_rate.sleep();
    }

    // When -1 is returned, the ronex with the given id is not present on the parameter server.
    int ronex_parameter_id = ronex::get_ronex_param_id(ronex_id);
    if ( ronex_parameter_id == -1 )
      {
        ROS_INFO( "Did not find the ronex with ronex_id %s.", ronex_id.c_str() );
        return false; // Failed to set path.
      }
    
    // The ronex is present on the parameter server and ronex_parameter_id
    // contains the id on which the given ronex is stored on the parameter server.

    std::string path_key = get_key_( ronex_parameter_id, std::string("path") );
    ros::param::get( path_key, path );

    return true; // Path is set.
  }
  
private:
  /**
   * Convert the given integer into a string.
   *
   * @param d The integer to be converted.
   * @return The integer as a string. 
   **/
  std::string to_string_(int d)
  {
    std::stringstream ss;
    ss << d;
    std::string s(ss.str());
    return s;
  }
  
  /**
   * Construct key for ros::param::get.
   *
   * @param ronex_parameter_id Part of the key.
   * @param part Part of the key.
   * @return The key.
   **/
  std::string get_key_(int ronex_parameter_id, std::string part)
  {
    std::string key("/ronex/devices/");
    key += to_string_(ronex_parameter_id);
    key += "/";
    key += part;
    return key;
  }
};

//-------------------------------------------------------------------------------

/**
 * Dim a LED light with PWM. It takes 10 seconds.
 *
 * @param n A ROS node handle.
 * @param topic For example "/ronex/general_io/1/command/0".
 */
void dimLED( ros::NodeHandle& n, const std::string& topic )
{
  ros::Publisher pub = n.advertise<sr_ronex_msgs::PWM>( topic, 1000 );
 
  // Set the switching frequency to 100kHz.
  short unsigned int pwm_period = 320;
  // Start with a 100% duty cycle.
  short unsigned int pwm_on_time_0 = pwm_period;
  // The second output is not used.
  short unsigned int pwm_on_time_1 = 0;
  
  // We want to run at 10Hz.
  ros::Rate loop_rate(10); 
  while (ros::ok())
    {   
      // Dim the light...
      pwm_on_time_0 -= 3;
      if (pwm_on_time_0 == 0 || pwm_on_time_0 > pwm_period)
	pwm_on_time_0 = pwm_period;
	  
      sr_ronex_msgs::PWM msg;
      msg.pwm_period    = pwm_period;
      msg.pwm_on_time_0 = pwm_on_time_0;
      msg.pwm_on_time_1 = pwm_on_time_1;
      
      // The publish() function sends the message. 
      // The parameter is the message object.
      pub.publish(msg);
      
      ros::spinOnce();
      loop_rate.sleep();
    }
}

//-------------------------------------------------------------------------------

/**
 * This example demonstrates how to dim a LED light with pulse-width modulation (PWM). 
 **/
int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "sr_ronex_dim_LED_with_PWM");
  
  // Create a handle to this process' node. 
  ros::NodeHandle n;
  
  // Get the path of the ronex with the given ronex id (i.e., "1").
  SrRonexExample example;
  std::string ronex_id("1"), path;
  example.get_ronex_path_( ronex_id, path );
 
  // Use the first PWM output to dim the LED light.
  // For example "/ronex/general_io/1" + "/command/0".
  std::string topic = path + "/command/0"; 
  dimLED( n, topic );

  return 0;
}

//-------------------------------------------------------------------------------
