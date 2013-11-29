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
    post_init_();
  }

  SPIPassthroughController::~SPIPassthroughController()
  {}

  bool SPIPassthroughController::command_srv_cb( sr_ronex_msgs::SPI::Request &req,
                                                 sr_ronex_msgs::SPI::Response &res,
                                                 size_t spi_out_index )
  {
    //transmitting the bytes we received
    standard_commands_[spi_out_index]->packet.num_bytes = static_cast<int8u>(req.data.size());

    ROS_ERROR_STREAM("From passthrough: received "<< req.data.size()<<"bytes: ");
    for(size_t i = 0; i < req.data.size(); ++i)
    {
      ROS_ERROR_STREAM("    ["<<i<<"] -> " << static_cast<int>(req.data[i]) );
      standard_commands_[spi_out_index]->packet.data_bytes[i] = static_cast<int8u>(req.data[i]);
    }

    //pushing to the command queue to be sent through etherCAT
    command_queue_[spi_out_index].push(standard_commands_[spi_out_index]);

    //wait for the response to be received
    bool not_received = true;
    while( not_received )
    {
      //sleep roughly 1ms to wait for new etherCAT packet to be received.
      usleep(1000);

      for(size_t i = 0; i < status_queue_.size(); ++i)
      {
        if( status_queue_[i].front().first == standard_commands_[spi_out_index] )
        {
          //found the status command corresponding to the command we sent
          // updating the response
          for(size_t j = 0; j < req.data.size(); ++j)
          {
            res.data.push_back(status_queue_[i].front().second->data_bytes[j]);
          }
          not_received = false;
          break;
        }
      }
    }

    return true;
  }

  void SPIPassthroughController::post_init_()
  {
    /*dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig>(ros::NodeHandle(topic_prefix_)));
    function_cb_ = boost::bind(&SPIPassthroughController::dynamic_reconfigure_cb, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(function_cb_);
    */

    for(size_t i = 0; i < NUM_SPI_OUTPUTS; ++i)
    {
      std::stringstream service_path;
      service_path << topic_prefix_ << "/command/passthrough/"<<i;

      ROS_ERROR_STREAM("prefix["<<i<<"] = " <<topic_prefix_ << "  -> " << service_path.str());

      command_srv_ = node_.advertiseService<sr_ronex_msgs::SPI::Request, sr_ronex_msgs::SPI::Response>(service_path.str(), boost::bind(&SPIPassthroughController::command_srv_cb, this, _1, _2,  i));

      standard_commands_.push_back(boost::shared_ptr<SplittedSPICommand>(new SplittedSPICommand()));
    }
  }

  void SPIPassthroughController::dynamic_reconfigure_cb(sr_ronex_drivers::SPIConfig &config, uint32_t level)
  {
    spi_->command_->command_type = static_cast<int16u>(config.command_type);

    ROS_ERROR("@TODO: add more config");
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
