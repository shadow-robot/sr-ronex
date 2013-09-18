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

#include <ros/ros.h>
#include <ros/console.h>
#include <list>

class SR_RONEX_Example
{
public:
  SR_RONEX_Example(ros::NodeHandle * nh)
    : _nh(nh) {}
  ~SR_RONEX_Example() {}
  
public:
  void run(void)
  {
    this->find_ronexes();
    this->set_param();
  }

private:
  void find_ronexes(void)
  {
    ros::Rate loop_rate(10);
    std::string param;
    while ( _nh->getParam("/ronex/0/ronex_id", param ) == false ) {
      ROS_INFO("Waiting for the ronex to be loaded properly.");
      loop_rate.sleep();
    }

    std::string ronex_param;
    if ( _nh->getParam("/ronex", ronex_param) == true) {
      
      int ronex_id;
      _ronex_ids.push_back(ronex_id);
    }
  }

  void set_param(void)
  {

  }

private:
  ros::NodeHandle * _nh;
  std::list<int>    _ronex_ids;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "parse_parameter_servers");

  ROS_INFO("Hello %s", "World");

  ros::NodeHandle nh;

  SR_RONEX_Example example( &nh );
  example.run();

  return 0;
}
