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

PLUGINLIB_EXPORT_CLASS(ronex::SPIPassthroughController, controller_interface::ControllerBase)

namespace ronex
{
bool SPIPassthroughController::init(ros_ethercat_model::RobotStateInterface* robot, ros::NodeHandle &n)
{
  if ( !pre_init_(robot, n) )
    return false;

  standard_commands_.assign(NUM_SPI_OUTPUTS, SplittedSPICommand());
  for (size_t i = 0; i < NUM_SPI_OUTPUTS; ++i)
  {
    std::ostringstream service_path;
    service_path << topic_prefix_ << "/command/passthrough/" << i;

    command_srv_.push_back(
            node_.advertiseService<sr_ronex_msgs::SPI::Request, sr_ronex_msgs::SPI::Response>(service_path.str(),
                                                  boost::bind(&SPIPassthroughController::command_srv_cb,
                                                              this, _1, _2,  i)) );
  }

  dynamic_reconfigure_server_.reset(
          new dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig>(ros::NodeHandle(topic_prefix_)));
  function_cb_ = boost::bind(&SPIPassthroughController::dynamic_reconfigure_cb, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(function_cb_);

  return true;
}

bool SPIPassthroughController::command_srv_cb(sr_ronex_msgs::SPI::Request &req,
                                              sr_ronex_msgs::SPI::Response &res,
                                              size_t spi_out_index)
{
  // transmitting the bytes we received
  standard_commands_[spi_out_index].packet.num_bytes = static_cast<int8u>(req.data.size());

  ROS_DEBUG_STREAM("From passthrough: received "<< req.data.size() << "bytes.");
  for (size_t i = 0; i < req.data.size(); ++i)
  {
    try
    {
      standard_commands_[spi_out_index].packet.data_bytes[i] = static_cast<int8u>(req.data[i]);
    }
    catch(...)
    {
      ROS_ERROR_STREAM("Input[" << i << "]: " << req.data[i] << " could not be converted to int");
    }
  }

  // pushing to the command queue to be sent through etherCAT
  command_queue_[spi_out_index].push(standard_commands_[spi_out_index]);

  // wait for the response to be received
  bool not_received = true;
  while ( not_received )
  {
    // sleep roughly 1ms to wait for new etherCAT packet to be received.
    usleep(1000);

    // check if the status_queue has the same command and that the response has been received
    if (status_queue_[spi_out_index].size() > 0 &&
        std::equal(status_queue_[spi_out_index].front().first.packet.data_bytes,
        status_queue_[spi_out_index].front().first.packet.data_bytes +
        sizeof status_queue_[spi_out_index].front().first.packet.data_bytes /
        sizeof *status_queue_[spi_out_index].front().first.packet.data_bytes,
        standard_commands_[spi_out_index].packet.data_bytes) &&
        status_queue_[spi_out_index].front().second.received == true)
    {
      // found the status command corresponding to the command we sent
      // updating the response
      for (size_t j = 0; j < req.data.size(); ++j)
      {
        std::ostringstream hex;
        try
        {
          hex << static_cast<unsigned int>(status_queue_[spi_out_index].front().second.packet.data_bytes[j]);
        }
        catch(...)
        {
          ROS_ERROR_STREAM("Can't cast to uint.");
          hex << "bad_data";
        }
        res.data.push_back(hex.str());
      }
      not_received = false;

      // we used the status (sent it back to the user through the service
      // response -> set flag to pop status from the queue
      delete_status_[spi_out_index] = true;
      break;
    }
  }
  return true;
}

void SPIPassthroughController::dynamic_reconfigure_cb(sr_ronex_drivers::SPIConfig &config, uint32_t level)
{
  spi_->command_->command_type = static_cast<int16u>(config.command_type);

  // setting up spi 0
  standard_commands_[0].packet.clock_divider = static_cast<int16u>(config.spi_0_clock_divider);
  standard_commands_[0].packet.SPI_config = 0;
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_mode_0);
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_0_input_trigger);
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_0_mosi_somi);
  standard_commands_[0].packet.inter_byte_gap = static_cast<int16u>(config.spi_0_inter_byte_gap);

  // setting up spi 1
  standard_commands_[1].packet.clock_divider = static_cast<int16u>(config.spi_1_clock_divider);
  standard_commands_[1].packet.SPI_config = 0;
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_mode_1);
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_1_input_trigger);
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_1_mosi_somi);
  standard_commands_[1].packet.inter_byte_gap = static_cast<int16u>(config.spi_1_inter_byte_gap);

  // setting up spi 2
  standard_commands_[2].packet.clock_divider = static_cast<int16u>(config.spi_2_clock_divider);
  standard_commands_[2].packet.SPI_config = 0;
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_mode_2);
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_2_input_trigger);
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_2_mosi_somi);
  standard_commands_[2].packet.inter_byte_gap = static_cast<int16u>(config.spi_2_inter_byte_gap);

  // setting up spi 3
  standard_commands_[3].packet.clock_divider = static_cast<int16u>(config.spi_3_clock_divider);
  standard_commands_[3].packet.SPI_config = 0;
  standard_commands_[3].packet.SPI_config |= static_cast<int16u>(config.spi_mode_3);
  standard_commands_[3].packet.SPI_config |= static_cast<int16u>(config.spi_3_input_trigger);
  standard_commands_[3].packet.SPI_config |= static_cast<int16u>(config.spi_3_mosi_somi);
  standard_commands_[3].packet.inter_byte_gap = static_cast<int16u>(config.spi_3_inter_byte_gap);


  if ( config.pin_output_state_pre_DIO_0 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_0;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_0;

  if ( config.pin_output_state_post_DIO_0 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_0;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_0;


  if ( config.pin_output_state_pre_DIO_1 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_1;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_1;

  if ( config.pin_output_state_post_DIO_1 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_1;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_1;


  if ( config.pin_output_state_pre_DIO_2 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_2;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_2;

  if ( config.pin_output_state_post_DIO_2 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_2;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_2;


  if ( config.pin_output_state_pre_DIO_3 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_3;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_3;

  if ( config.pin_output_state_post_DIO_3 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_3;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_3;


  if ( config.pin_output_state_pre_DIO_4 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_4;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_4;

  if ( config.pin_output_state_post_DIO_4 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_4;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_4;


  if ( config.pin_output_state_pre_DIO_5 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_DIO_5;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_5;

  if ( config.pin_output_state_post_DIO_5 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_DIO_5;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_DIO_5;


  if ( config.pin_output_state_pre_CS_0 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_CS_0;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_0;

  if ( config.pin_output_state_post_CS_0 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_CS_0;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_0;


  if ( config.pin_output_state_pre_CS_1 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_CS_1;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_1;

  if ( config.pin_output_state_post_CS_1 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_CS_1;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_1;


  if ( config.pin_output_state_pre_CS_2 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_CS_2;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_2;

  if ( config.pin_output_state_post_CS_2 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_CS_2;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_2;


  if ( config.pin_output_state_pre_CS_3 )
    cmd_pin_output_states_pre_ |= PIN_OUTPUT_STATE_CS_3;
  else
    cmd_pin_output_states_pre_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_3;

  if ( config.pin_output_state_post_CS_3 )
    cmd_pin_output_states_post_ |= PIN_OUTPUT_STATE_CS_3;
  else
    cmd_pin_output_states_post_ &= 0xFFFF - PIN_OUTPUT_STATE_CS_3;
}

}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
