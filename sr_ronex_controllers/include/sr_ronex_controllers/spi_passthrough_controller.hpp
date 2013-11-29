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
 * @file   spi_passthrough_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A passthrough for the SPI RoNeX module: sends one command through a service
 * and returns the corresponding response.
 **/

#ifndef _SPI_PASSTHROUGH_CONTROLLER_H_
#define _SPI_PASSTHROUGH_CONTROLLER_H_

#include <ros/node_handle.h>

#include "sr_ronex_controllers/spi_base_controller.hpp"
#include <sr_ronex_msgs/SPI.h>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/SPIConfig.h"

namespace ronex
{
  class SPIPassthroughController
    : public SPIBaseController
  {
  public:
    SPIPassthroughController();
    virtual ~SPIPassthroughController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    bool command_srv_cb( sr_ronex_msgs::SPI::Request &req,
                         sr_ronex_msgs::SPI::Response &res,
                         size_t spi_out_index );

    void dynamic_reconfigure_cb(sr_ronex_drivers::SPIConfig &config, uint32_t level);

  private:
    std::vector<ros::ServiceServer> command_srv_;

    //vector containing one command per spi output.
    // Some parameters of these commands are updated through the dynamic reconfigure interface
    // The data packet is updated from the service.
    std::vector<boost::shared_ptr<SplittedSPICommand> > standard_commands_;

    ///Dynamic reconfigure server for setting the parameters of the driver
    boost::shared_ptr<dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig> > dynamic_reconfigure_server_;
    dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig>::CallbackType function_cb_;

    //Instantiating the services / dynamic reconfigure callbacks etc..
    void post_init_();
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_PASSTHROUGH_CONTROLLER_H_ */
