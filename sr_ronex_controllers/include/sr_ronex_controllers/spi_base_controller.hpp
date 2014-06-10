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

#include <pr2_controller_interface/controller.h>
#include <sr_ronex_hardware_interface/spi_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>
#include <queue>

namespace ronex
{
  struct SplittedSPICommand
  {
    SPI_PACKET_OUT packet;

    SplittedSPICommand()
    {}

    SplittedSPICommand(SplittedSPICommand* copy_me)
    {
      this->packet = copy_me->packet;
    }
  };

  class SPIBaseController
    : public pr2_controller_interface::Controller
  {
  public:
    SPIBaseController();
    virtual ~SPIBaseController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

  protected:
    ros::NodeHandle node_;

    ///prefix used for creating topics / services
    std::string topic_prefix_;

    int loop_count_;

    ronex::SPI* spi_;

    std::vector<std::queue<SplittedSPICommand*> > command_queue_;
    std::vector<std::queue<std::pair<SplittedSPICommand*, SPI_PACKET_IN* > > > status_queue_;

    uint16_t     cmd_pin_output_states_pre_;
    uint16_t     cmd_pin_output_states_post_;

    bool pre_init_(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    void copy_splitted_to_cmd_(uint16_t spi_index);
  private:
    bool new_command;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_BASE_CONTROLLER_H_ */
