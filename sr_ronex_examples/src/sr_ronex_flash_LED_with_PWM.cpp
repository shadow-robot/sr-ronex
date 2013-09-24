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
 * @file   sr_ronex_flash_LED_with_PWM.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Demonstrate how to flash a LED light with RoNeX.
 **/

//-------------------------------------------------------------------------------

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/lexical_cast.hpp>

#include "sr_ronex_msgs/PWM.h"
#include "sr_ronex_utilities/sr_ronex_utilities.hpp"

//-------------------------------------------------------------------------------

/**
 * This class demonstrates how to find the General I/O module with the given ronex_id.
 **/
class SrRonexFindGeneralIOModule
{
public:
  SrRonexFindGeneralIOModule() {}
  ~SrRonexFindGeneralIOModule() {}
  
public:
  /**
   * Find the path of the General I/O module with the given ronex_id.
   *
   * @param ronex_id_as_int Select the General I/O module.
   * @param path The path of the module.
   * @return True if the module is found and the path is set. Otherwise, false.
   **/
  bool get_path_( const short unsigned int& ronex_id_as_int, std::string& path )
  {
    std::string ronex_id = this->to_string_(ronex_id_as_int);
    
    // Wait until there's at least one General I/O module.
    ros::Rate loop_rate(10);
    std::string param;
    while ( ros::param::get("/ronex/devices/0/ronex_id", param ) == false )
    {
      ROS_INFO_STREAM( "Waiting for General I/O module to be loaded properly.\n" );
      loop_rate.sleep();
    }
    
    // When -1 is returned, the module with the given id is not present on the parameter server. 
    // Note that ronex parameter id starts from zero.
    int ronex_parameter_id = ronex::get_ronex_param_id(ronex_id);
    if ( ronex_parameter_id == -1 )
    {
      ROS_ERROR_STREAM( "Did not find the General I/O module with ronex_id " << ronex_id << ".\n" );
      return false; // Failed to set path.
    }
    
    // The module is present on the parameter server and ronex_parameter_id
    // contains the id on which the module is stored on the parameter server.
    
    std::string path_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("path") );
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
    return boost::lexical_cast<std::string>(d);
  }
};

//-------------------------------------------------------------------------------

/**
 * Flash a LED light with PWM.
 *
 * @param n A ROS node handle.
 * @param topic For example "/ronex/general_io/1/command/0".
 */
void flash_LED( ros::NodeHandle& n, const std::string& topic )
{
  ros::Publisher pub = n.advertise<sr_ronex_msgs::PWM>( topic, 1000 );
  
  short unsigned int pwm_period = 320;
  // Start with a 100% duty cycle.
  short unsigned int pwm_on_time_0 = pwm_period;
  // The second output is not used.
  short unsigned int pwm_on_time_1 = 0;
  
  ros::Rate loop_rate(100); 
  while ( ros::ok() )
  {   
    // Flash the light...
    pwm_on_time_0 -= 10;
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
 * Assume that your RoNeX consists of a Bridge (IN) module, and one 
 * or multiple General I/O module(s). This example demonstrates how 
 * to flash a LED light with pulse-width modulation (PWM). 
 **/
int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "sr_ronex_flash_LED_with_PWM");
  
  // Create a handle to this process' node. 
  ros::NodeHandle n;
  
  // Get the path of the General I/O module with the given ronex id.
  // Note that you may have to set the value of ronex_id,
  // depending on which General I/O board the LED is connected to.
  short unsigned int ronex_id; 
  std::cout << "Please enter the ronex id: ";
  std::cin >> ronex_id;
  std::string path;
  SrRonexFindGeneralIOModule findModule;
  if ( findModule.get_path_( ronex_id, path ) ) 
  {
    // Always use the first digital I/O channel to flash the LED light.
    // For example "/ronex/general_io/1" + "/command/pwm/0".
    std::string topic = path + "/command/pwm/0"; 
    ROS_INFO_STREAM( "Topic = " << topic << "\n" );
    flash_LED( n, topic );
  }
  
  return 0;
}

//-------------------------------------------------------------------------------
