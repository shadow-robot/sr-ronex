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
 * @file   sr_ronex_parse_parameter_server.cpp
 * @author Yi Li <yi@shadowrobot.com>
 * @brief  Parse data from parameter server for ronexes.
 **/

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sstream>

#include "sr_ronex_utilities/sr_ronex_utilities.hpp"

//-------------------------------------------------------------------------------

 /**
  * Assume that your RoNeX consists of a Bridge (IN) module, and one or multiple
  * General I/O module(s). This class demonstrates how to access the General I/O module(s)
  * listed in the parameter server. For each General I/O module, the parameter server 
  * stores parameters such as its product_id, product_name, ronex_id, path, and serial.
  **/
class SrRonexParseParamExample
{
public:
  SrRonexParseParamExample() 
  {
    find_general_io_modules_();
  }
  
  ~SrRonexParseParamExample() {}
  
private:

 /**
  * Find all General I/O modules listed in the parameter server.
  * Note that the help method ronex::get_ronex_param_id checks 
  * the General I/O modules already present on the parameter server and returns 
  * an id on which the given module is stored on the parameter server.
  **/
  void find_general_io_modules_(void)
  {
    // Wait until there's at least one General I/O module.
    ros::Rate loop_rate(10);
    std::string param;
    while ( ros::param::get("/ronex/devices/0/ronex_id", param ) == false )
    {
      ROS_INFO( "Waiting for General I/O module to be loaded properly." );
      loop_rate.sleep();
    }
    
    // The maximum number of General I/O modules is 5.
    for (int k = 1; k <= 5; k++)
    {
      // Assume that alias has not been set, and hence ronex_id is equal to serial.
      // Note that serial starts from 1.
      std::string curr_ronex_id = to_string_(k);
      
      // When -1 is returned, the module with the given id is not present on the parameter server.
      int ronex_parameter_id = ronex::get_ronex_param_id(curr_ronex_id);
      if ( ronex_parameter_id == -1 )
        continue;
      
      // The module is present on the parameter server and ronex_parameter_id
      // contains the id on which the module is stored on the parameter server.
      
      // Retrieve the values of all parameters related to the current module.
      std::string product_id;
      std::string product_id_key = get_key_( ronex_parameter_id, std::string("product_id") );
      ros::param::get( product_id_key, product_id );
      
      std::string product_name;
      std::string product_name_key = get_key_( ronex_parameter_id, std::string("product_name") );
      ros::param::get( product_name_key, product_name );
      
      // Note that ronex_id is equal to curr_ronex_id.
      std::string ronex_id;
      std::string ronex_id_key = get_key_( ronex_parameter_id, std::string("ronex_id") );
      ros::param::get( ronex_id_key, ronex_id );
      
      std::string path;
      std::string path_key = get_key_( ronex_parameter_id, std::string("path") );
      ros::param::get( path_key, path );
      
      std::string serial;
      std::string serial_key = get_key_( ronex_parameter_id, std::string("serial") );
      ros::param::get( serial_key, serial );
      
      ROS_INFO( "*** General I/O module %d ***",  ronex_parameter_id );
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
  ros::init(argc, argv, "sr_ronex_parse_parameter_server");
  
  // Create a handle to this process' node. 
  ros::NodeHandle n;
  
  // This class demonstrates how to access the General I/O module(s) 
  // listed in the parameter server. 
  SrRonexParseParamExample example;
  
  return 0;
}

//-------------------------------------------------------------------------------
