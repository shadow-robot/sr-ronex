/*
 * Copyright (c) 2016, Shadow Robot Company, All rights reserved.
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
 * @file   spi_sensor_controller.h
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief  a controller to read the rotary sensor connected to SPI ronex
 **/
#ifndef SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H
#define SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H

#include <ros/node_handle.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <utility>
#include <vector>

#include "sr_ronex_controllers/spi_base_controller.hpp"
#include <sr_ronex_msgs/SPI.h>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/SPIConfig.h"
#include "std_msgs/Float64MultiArray.h"

namespace ronex
{
class SPISensorReadController
  : public SPIBaseController
{
public:
  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  void update(const ros::Time&, const ros::Duration&);
  std::vector<double> get_sensor_value();
  std::vector<int> get_spi_channel();


private:
  std::vector<int> spi_channel_;
  static const int default_spi_channel_;
  static const size_t sensor_message_length_;
  static const size_t spi_mode_;
  static const double publish_rate_;


  std_msgs::Float64MultiArray sensor_msg_;
  std::vector<ros::ServiceServer> command_srv_;
  realtime_tools::RealtimePublisher<std_msgs::Float64MultiArray> sensor_data_publisher_;
  // vector containing one command per spi output.
  // Some parameters of these commands are updated through the dynamic reconfigure interface
  // The data packet is updated from the service.
  std::vector<SplittedSPICommand> standard_commands_;

  std::vector<uint16_t> chip_select_masks_;
  bool first_run_;
  ros::Time last_publish_time_;
};
}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif  // SR_RONEX_CONTROLLERS_SPI_SENSOR_READ_CONTROLLER_H
