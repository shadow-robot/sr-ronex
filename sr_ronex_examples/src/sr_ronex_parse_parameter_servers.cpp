/**
* @file sr_ronex_parse_parameter_servers.cpp
* @author Yi Li <yi@shadowrobot.com>
*
* Copyright 2013 Shadow Robot Company Ltd.
*
* This program is Proprietary software: you cannot redistribute it or modify it
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.
*
*
* @brief Parse parameter servers for ronexes.
*
*
*/

//-------------------------------------------------------------------------------

#include <ros/ros.h>
#include <ros/console.h>
#include <string>
#include <sstream>

#include "sr_ronex_utilities.hpp"

//-------------------------------------------------------------------------------

class SR_RONEX_Example
{
public:
  SR_RONEX_Example() {}
  ~SR_RONEX_Example() {}
  
public:
  void run(void)
  {
    this->find_ronexes();
  }

private:
  void find_ronexes(void)
  { 
    ros::Rate loop_rate(10);
    std::string param;
    while ( ros::param::get("/ronex/0/ronex_id", param ) == false ) {
      ROS_INFO( "Waiting for the ronex to be loaded properly." );
      loop_rate.sleep();
    }
    
    /* 
     * Checks the ronexes already present on the parameter server and returns 
     * an id on which the given ronex is stored on the parameter server.
     * The method returns the index of the ronex in the parameter server. 
     * -1 if not found. Or the next free index if ronex_id == "".
     */
    std::string ronex_id("");
    int next_free_index = ronex::get_ronex_param_id(ronex_id);
    for (int k = 0; k < next_free_index; k++)
      {
	ronex_id = this->to_string(k);
	int ronex_parameter_id = ronex::get_ronex_param_id(ronex_id);
	if ( ronex_parameter_id == -1 ) {
	  ROS_INFO( "Did not find the ronex with ronex_id %s.", ronex_id.c_str() );
	  continue;
	}
	
	std::string product_id;
	std::string product_id_key = this->get_key( ronex_parameter_id, std::string("product_id") );
	ros::param::get( product_id_key, product_id );
	ROS_INFO( "product_id = %s.", product_id.c_str() );
	
	std::string product_name;
	std::string product_name_key = this->get_key( ronex_parameter_id, std::string("product_name") );
	ros::param::get( product_name_key, product_name );
	ROS_INFO( "product_name = %s.", product_name.c_str() );
	
	std::string ronex_id;
	std::string ronex_id_key = this->get_key( ronex_parameter_id, std::string("ronex_id") );
	ros::param::get( ronex_id_key, ronex_id );
	ROS_INFO( "ronex_id = %s.", ronex_id.c_str() );
	
	std::string path;
	std::string path_key = this->get_key( ronex_parameter_id, std::string("path") );
	ros::param::get( path_key, path );
	ROS_INFO( "path = %s.", path.c_str() );
	
	std::string serial;
	std::string serial_key = this->get_key( ronex_parameter_id, std::string("serial") );
	ros::param::get( serial_key, serial );
	ROS_INFO( "serial = %s.", serial.c_str() );		
      }
  }
  
  // Convert integer d to string s.
  std::string to_string(int d) 
  {
    std::stringstream ss; 
    ss << d; 
    std::string s(ss.str());
    return s;
  }

  std::string get_key(int ronex_parameter_id, std::string part)
  {
    std::string key("/ronex/");
    key += this->to_string(ronex_parameter_id);
    key += "/";
    key += part;
    return key;
  }
};

//-------------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parse_parameter_servers");

  SR_RONEX_Example example;
  example.run();

  return 0;
}

//-------------------------------------------------------------------------------
