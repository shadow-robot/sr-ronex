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
 * @file   spi_base_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A base controller for the SPI module.
 **/

#ifndef _SPI_BASE_CONTROLLER_H_
#define _SPI_BASE_CONTROLLER_H_

#include <ros/node_handle.h>

#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <sr_ronex_hardware_interface/spi_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>
#include <queue>
#include <utility>
#include <string>
#include <vector>
#include <boost/circular_buffer.hpp>

#define NUM_BUFFER_ELEMENTS 100

namespace ronex
{
struct SplittedSPICommand
{
  SPI_PACKET_OUT packet;

  SplittedSPICommand() : packet()
  {}

  explicit SplittedSPICommand(SplittedSPICommand* copy_me)
  {
    this->packet = copy_me->packet;
  }
};

struct SPIResponse
{
  bool received;
  int loop_number;
  SPI_PACKET_IN packet;
};

class SPIBaseController
  : public controller_interface::Controller<ros_ethercat_model::RobotState>
{
public:
  SPIBaseController();

  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  virtual void starting(const ros::Time&);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time&, const ros::Duration&);

protected:
  ros::NodeHandle node_;

  /// prefix used for creating topics / services
  std::string topic_prefix_;

  int loop_count_;

  ronex::SPI* spi_;

  std::vector<std::queue<int, boost::circular_buffer<SplittedSPICommand> > > command_queue_;
  std::vector<std::queue<int, boost::circular_buffer<std::pair<SplittedSPICommand, SPIResponse > > > > status_queue_;

  uint16_t     cmd_pin_output_states_pre_;
  uint16_t     cmd_pin_output_states_post_;

  bool pre_init_(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  void copy_splitted_to_cmd_(uint16_t spi_index);
protected:
  // set to true when response have been processed and it is ready to be deleted
  std::vector<bool> delete_status_;
};
}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_BASE_CONTROLLER_H_ */
