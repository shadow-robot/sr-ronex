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

#include <string>
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/lexical_cast.hpp>

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
      ROS_INFO_STREAM( "Waiting for General I/O module to be loaded properly.\n" );
      loop_rate.sleep();
    }
    
    std::string empty_ronex_id("");
    int next_ronex_parameter_id = ronex::get_ronex_param_id(empty_ronex_id);
    
    // ronex_parameter_id contains the id on which the module is stored on the parameter server.
    for (int ronex_parameter_id = 0; 
         ronex_parameter_id < next_ronex_parameter_id;
         ronex_parameter_id++)
    {  
      // Retrieve the values of all parameters related to the current module.
      std::string product_id;
      std::string product_id_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("product_id") );
      ros::param::get( product_id_key, product_id );
      
      std::string product_name;
      std::string product_name_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("product_name") );
      ros::param::get( product_name_key, product_name );

      // Path looks like "/ronex/general_io/2", where 2 is a ronex_id.
      std::string path;
      std::string path_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("path") );
      ros::param::get( path_key, path );
      
      std::string ronex_id;
      std::string ronex_id_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("ronex_id") );
      ros::param::get( ronex_id_key, ronex_id );

      std::string serial;
      std::string serial_key = ronex::get_ronex_devices_string( ronex_parameter_id, std::string("serial") );
      ros::param::get( serial_key, serial );
      
      ROS_INFO_STREAM( "*** General I/O module " << ronex_parameter_id << " ***" );
      ROS_INFO_STREAM( "product_id   = " << product_id );
      ROS_INFO_STREAM( "product_name = " << product_name );
      ROS_INFO_STREAM( "ronex_id     = " << ronex_id );
      ROS_INFO_STREAM( "path         = " << path );
      ROS_INFO_STREAM( "serial       = " << serial );
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
    return boost::lexical_cast<std::string>(d);
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
