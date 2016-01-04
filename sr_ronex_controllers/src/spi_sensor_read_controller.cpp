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
 * @file   spi_sensor_read_controller.cpp
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief  a controller to read the rotary sensor connected to SPI ronex
 **/
#include "sr_ronex_controllers/spi_sensor_read_controller.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"
#include <utility>

PLUGINLIB_EXPORT_CLASS(ronex::SPISensorReadController, controller_interface::ControllerBase)

namespace ronex
{
const size_t SPISensorReadController::spi_channel_ = 1;
const size_t SPISensorReadController::sensor_message_length_ = 2;
const size_t SPISensorReadController::spi_mode_ = 1;

bool SPISensorReadController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  if ( !pre_init_(robot, n) )
    return false;

  standard_commands_.assign(NUM_SPI_OUTPUTS, SplittedSPICommand());
  standard_commands_[spi_channel_].packet.num_bytes = sensor_message_length_;  // change this
  sensor_data_publisher_ = n.advertise<std_msgs::Float64>("sensor_message", 1);
  dynamic_reconfigure_server_.reset(
          new dynamic_reconfigure::Server<sr_ronex_drivers::SPIConfig>(ros::NodeHandle(topic_prefix_)));
  function_cb_ = boost::bind(&SPISensorReadController::dynamic_reconfigure_cb, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(function_cb_);
  first_run_ = true;
  return true;
}

void SPISensorReadController::update(const ros::Time& time, const ros::Duration& dur)
{
  if (first_run_)
  {
    first_run_ = false;
    sr_ronex_drivers::SPIConfig config;
    spi_->command_->command_type = static_cast<int16u>(1);

    // setting up spi
    standard_commands_[spi_channel_].packet.SPI_config = 1;
    standard_commands_[spi_channel_].packet.clock_divider = static_cast<int16u>(16);
    standard_commands_[spi_channel_].packet.SPI_config |= static_cast<int16u>(0);
    standard_commands_[spi_channel_].packet.SPI_config |= 0;
    standard_commands_[spi_channel_].packet.SPI_config |= 0;
    standard_commands_[spi_channel_].packet.inter_byte_gap = 0;
  }
  // the command will be sent at the end of the iteration,
  // removing the command from the queue but not freeing the
  // memory yet
  if ( status_queue_[spi_channel_].size() > 0)
  {
    if ( status_queue_[spi_channel_].front().second == NULL )
    {
      if (new_command)
      {
        new_command = false;
        spi_->nullify_command(spi_channel_);
      }

      // the response has not been received. If the command type is NORMAL
      // then the response can be updated (it's INVALID until the SPI responds)
      if ( spi_->state_->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL )
      {
        status_queue_[spi_channel_].front().second =
                new SPI_PACKET_IN(spi_->state_->info_type.status_data.spi_in[spi_channel_]);
        unsigned int high_byte =
            static_cast<unsigned int>(status_queue_[spi_channel_].front().second->data_bytes[0] & 0x3F);
        unsigned int low_byte = static_cast<unsigned int>(status_queue_[spi_channel_].front().second->data_bytes[1]);
        ROS_DEBUG_STREAM("sensor value is " << (high_byte << 8 | low_byte));
        sensor_msg_.data = static_cast<double>((high_byte << 8 | low_byte) *360) / 16384;
        sensor_data_publisher_.publish(sensor_msg_);
      }
    }
    status_queue_[spi_channel_].pop();
  }
  try
  {
    standard_commands_[spi_channel_].packet.data_bytes[0] = 0xFF;
    standard_commands_[spi_channel_].packet.data_bytes[1] = 0xFF;
    command_queue_[spi_channel_].push(&standard_commands_[spi_channel_]);
  }
  catch(...)
  {
    ROS_ERROR_STREAM("something bad happened");
  }
  try
  {
    status_queue_[spi_channel_].push(std::pair<SplittedSPICommand*, SPI_PACKET_IN*>());
    status_queue_[spi_channel_].front().first = command_queue_[spi_channel_].front();

    // now we copy the command to the hardware interface
    copy_splitted_to_cmd_(spi_channel_);

    new_command = true;
  }
  catch(...)
  {
    ROS_ERROR_STREAM("something really bad happened");
  }
  command_queue_[spi_channel_].pop();
}

void SPISensorReadController::dynamic_reconfigure_cb(sr_ronex_drivers::SPIConfig &config, uint32_t level)
{
  spi_->command_->command_type = static_cast<int16u>(config.command_type);

  // setting up spi 0
  standard_commands_[0].packet.clock_divider = static_cast<int16u>(config.spi_0_clock_divider);
  standard_commands_[0].packet.SPI_config = 1;
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_mode_0);
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_0_input_trigger);
  standard_commands_[0].packet.SPI_config |= static_cast<int16u>(config.spi_0_mosi_somi);
  standard_commands_[0].packet.inter_byte_gap = static_cast<int16u>(config.spi_0_inter_byte_gap);

  // setting up spi 1
  standard_commands_[1].packet.clock_divider = static_cast<int16u>(config.spi_1_clock_divider);
  standard_commands_[1].packet.SPI_config = 1;
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_mode_1);
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_1_input_trigger);
  standard_commands_[1].packet.SPI_config |= static_cast<int16u>(config.spi_1_mosi_somi);
  standard_commands_[1].packet.inter_byte_gap = static_cast<int16u>(config.spi_1_inter_byte_gap);

  // setting up spi 2
  standard_commands_[2].packet.clock_divider = static_cast<int16u>(config.spi_2_clock_divider);
  standard_commands_[2].packet.SPI_config = 1;
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_mode_2);
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_2_input_trigger);
  standard_commands_[2].packet.SPI_config |= static_cast<int16u>(config.spi_2_mosi_somi);
  standard_commands_[2].packet.inter_byte_gap = static_cast<int16u>(config.spi_2_inter_byte_gap);

  // setting up spi 3
  standard_commands_[3].packet.clock_divider = static_cast<int16u>(config.spi_3_clock_divider);
  standard_commands_[3].packet.SPI_config = 1;
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
} /* namespace ronex */
