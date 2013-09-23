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
 * @brief  Demonstrate how to read analogue data with RoNeX.
 **/

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sstream>

#include "sr_ronex_msgs/GeneralIOState.h"
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
      ROS_INFO( "Waiting for General I/O module to be loaded properly." );
      loop_rate.sleep();
    }

    // When -1 is returned, the module with the given id is not present on the parameter server. 
    // Note that ronex parameter id starts from zero.
    int ronex_parameter_id = ronex::get_ronex_param_id(ronex_id);
    if ( ronex_parameter_id == -1 )
      {
        ROS_INFO( "Did not find the General I/O module with ronex_id %s.", ronex_id.c_str() );
        return false; // Failed to set path.
      }
    
    // The module is present on the parameter server and ronex_parameter_id
    // contains the id on which the module is stored on the parameter server.

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
 * The callback function given to the subscribe() call.
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
 * This class demonstrates how to read analogue data with RoNeX.
 **/
int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "sr_ronex_read_analog_data");
  
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
      /**
       * The subscribe() call is how you tell ROS that you want to receive messages
       * on a given topic. Messages are passed to a callback function. The second
       * parameter to the subscribe() function is the size of the message queue.
       **/
      // For example "/ronex/general_io/1" + "/state"
      std::string topic = path + "/state"; 
      ros::Subscriber sub = n.subscribe( topic.c_str(), 
					 1000,
					 generalIOState_callback);
      
      /**
       * ros::spin() will enter a loop, pumping callbacks.  With this version, all
       * callbacks will be called from within this thread (the main one).  ros::spin()
       * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
       **/
      ros::spin();
    }

  return 0;
}

//-------------------------------------------------------------------------------
