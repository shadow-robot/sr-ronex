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

void generalIOStateCallback(const sr_ronex_msgs::GeneralIOState::ConstPtr& msg)
{
  ROS_INFO("I heard data");
}

//-------------------------------------------------------------------------------

 /**
  * This class demonstate how to use the ronexes listed in the parameter server.
  * For each ronex, the parameter server stores parameters such as 
  * its product_id, product_name, ronex_id, path, and serial.
  **/
class SrRonexExample
{
public:
  SrRonexExample() 
  {
    find_ronexes_();
  }
  
  ~SrRonexExample() {}
  
private:

 /**
  * Find all ronexes listed in the parameter server.
  * Note that the help method ronex::get_ronex_param_id checks 
  * the ronexes already present on the parameter server and returns 
  * an id on which the given ronex is stored on the parameter server.
  **/
  void find_ronexes_(void)
  {
    // Wait until ronexes are loaded.
    ros::Rate loop_rate(10);
    std::string param;
    while ( ros::param::get("/ronex/devices/0/ronex_id", param ) == false )
    {
      ROS_INFO( "Waiting for the ronex to be loaded properly." );
      loop_rate.sleep();
    }

    // ronex::get_ronex_param_id returns next free index, when ronex_id == "".
    std::string ronex_id("");
    int next_free_index = ronex::get_ronex_param_id(ronex_id);
    // Iterate through all ronexes.
    for (int k = 0; k < next_free_index; k++)
    {
      ronex_id = to_string_(k+1);
      // When -1 is returned, the ronex with the given id is not present on the parameter server.
      int ronex_parameter_id = ronex::get_ronex_param_id(ronex_id);
      if ( ronex_parameter_id == -1 )
      {
        ROS_INFO( "Did not find the ronex with ronex_id %s.", ronex_id.c_str() );
        continue;
      }

      // The ronex is present on the parameter server and ronex_parameter_id
      // contains the id on which the given ronex is stored on the parameter server.
            
      // Retrieve the values of all parameters related to the current ronex.
      std::string product_id;
      std::string product_id_key = get_key_( ronex_parameter_id, std::string("product_id") );
      ros::param::get( product_id_key, product_id );
     
      std::string product_name;
      std::string product_name_key = get_key_( ronex_parameter_id, std::string("product_name") );
      ros::param::get( product_name_key, product_name );
      
      std::string ronex_id;
      std::string ronex_id_key = get_key_( ronex_parameter_id, std::string("ronex_id") );
      ros::param::get( ronex_id_key, ronex_id );
      
      std::string path;
      std::string path_key = get_key_( ronex_parameter_id, std::string("path") );
      ros::param::get( path_key, path );
      
      std::string serial;
      std::string serial_key = get_key_( ronex_parameter_id, std::string("serial") );
      ros::param::get( serial_key, serial );

      ROS_INFO( "*** Ronex %d ***",  ronex_parameter_id );
      ROS_INFO( "product_id   = %s", product_id.c_str() );
      ROS_INFO( "product_name = %s", product_name.c_str() );
      ROS_INFO( "ronex_id     = %s", ronex_id.c_str() );
      ROS_INFO( "path         = %s", path.c_str() );
      ROS_INFO( "serial       = %s", serial.c_str() );
    }
  }
  
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

int main(int argc, char **argv)
{
  // Initialize ROS with a unique node name.
  ros::init(argc, argv, "sr_ronex_read_analog_data");

  // Create a handle to this process' node. 
  ros::NodeHandle n;
  
  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic. Messages are passed to a callback function, here
   * called chatterCallback. The second parameter to the subscribe() function is 
   * the size of the message queue.
   **/
  ros::Subscriber sub = n.subscribe( "/ronex/general_io/1/state", 
				     1000,
				     generalIOStateCallback);
  
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   **/
  ros::spin();
  
  return 0;
}

//-------------------------------------------------------------------------------
