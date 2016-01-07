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
 * @file   spi_sensor_read_controller.cpp
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief  a controller to read the rotary sensor connected to SPI ronex
 **/
#include "sr_ronex_controllers/spi_sensor_read_controller.h"
#include "pluginlib/class_list_macros.h"
#include "std_msgs/Float64.h"
#include <utility>
#include <stdexcept>
#include <string>

PLUGINLIB_EXPORT_CLASS(ronex::SPISensorReadController, controller_interface::ControllerBase)

namespace ronex
{
const int SPISensorReadController::default_spi_channel_ = 1;
const size_t SPISensorReadController::sensor_message_length_ = 2;
const size_t SPISensorReadController::spi_mode_ = 1;

bool SPISensorReadController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  if ( !pre_init_(robot, n) )
    return false;
  chip_select_masks_.resize(NUM_SPI_OUTPUTS);
  chip_select_masks_[0] = PIN_OUTPUT_STATE_CS_0;
  chip_select_masks_[1] = PIN_OUTPUT_STATE_CS_1;
  chip_select_masks_[2] = PIN_OUTPUT_STATE_CS_2;
  chip_select_masks_[3] = PIN_OUTPUT_STATE_CS_3;
  n.param<int>("SPI_sensor_channel", spi_channel_, default_spi_channel_);
  if (spi_channel_ < 0 || spi_channel_ > NUM_SPI_OUTPUTS)
  {
    throw std::invalid_argument("spi channel parameter should be larger than or equal 0 "
        "and smaller than number of SPI outputs");
  }
  standard_commands_.assign(NUM_SPI_OUTPUTS, SplittedSPICommand());
  standard_commands_[spi_channel_].packet.num_bytes = sensor_message_length_;
  std::string ronex_id;
  node_.getParam("ronex_id", ronex_id);
  std::stringstream topic_name;
  topic_name << "sensor_message_" << ronex_id << "_" << spi_channel_;
  sensor_data_publisher_ = n.advertise<std_msgs::Float64>(topic_name.str(), 1);

  first_run_ = true;

  return true;
}

void SPISensorReadController::update(const ros::Time& time, const ros::Duration& dur)
{
  if (first_run_)  // configuring the channel for communication
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
    cmd_pin_output_states_post_ |= chip_select_masks_[spi_channel_];
  }
  // the command will be sent at the end of the iteration,
  // removing the command from the queue but not freeing the
  // memory yet
  if (status_queue_[spi_channel_].size() > 0)
  {
    if (status_queue_[spi_channel_].front().second == NULL)
    {
      if (new_command)
      {
        new_command = false;
        spi_->nullify_command(spi_channel_);
      }

      // the response has not been received. If the command type is NORMAL
      // then the response can be updated (it's INVALID until the SPI responds)
      if (spi_->state_->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL)
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
    ROS_ERROR_STREAM("error while copy_splitted_to_cmd_");
  }
  command_queue_[spi_channel_].pop();
}
double SPISensorReadController::get_sensor_value()
{
  return sensor_msg_.data;
}
} /* namespace ronex */
