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

namespace ronex
{
  class SPIPassthroughController
    : public SPIBaseController
  {
  public:
    SPIPassthroughController();
    virtual ~SPIPassthroughController();

    bool command_srv_cb( sr_ronex_msgs::SPI::Request &req,
                         sr_ronex_msgs::SPI::Response &res,
                         size_t spi_out_index );

  private:
    ros::ServiceServer command_srv_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_PASSTHROUGH_CONTROLLER_H_ */
