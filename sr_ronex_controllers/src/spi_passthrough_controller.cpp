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

#include "sr_ronex_controllers/spi_passthrough_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::SPIPassthroughController, pr2_controller_interface::Controller)

namespace ronex
{
  SPIPassthroughController::SPIPassthroughController()
    : SPIBaseController()
  {
    for(size_t i = 0; i < NUM_SPI_OUTPUTS; ++i)
    {
      std::stringstream service_path;
      service_path << topic_prefix_ << "/command/passthrough/"<<i;
      command_srv_ = node_.advertiseService<sr_ronex_msgs::SPI::Request, sr_ronex_msgs::SPI::Response>(service_path.str(), boost::bind(&SPIPassthroughController::command_srv_cb, this, _1, _2,  i));
    }
  }

  SPIPassthroughController::~SPIPassthroughController()
  {}

  bool SPIPassthroughController::command_srv_cb( sr_ronex_msgs::SPI::Request &req,
                                                 sr_ronex_msgs::SPI::Response &res,
                                                 size_t spi_out_index )
  {
    ROS_ERROR("TODO");
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
