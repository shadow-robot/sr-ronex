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
#include "std_msgs/Float64MultiArray.h"
#include <utility>
#include <stdexcept>
#include <string>
#include <vector>

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
  std::string ronex_id;
  node_.getParam("ronex_id", ronex_id);
  std::stringstream topic_name;
  topic_name << "sensor_message_" << ronex_id;
  standard_commands_.assign(NUM_SPI_OUTPUTS, SplittedSPICommand());

  if (!n.hasParam("SPI_sensor_channel"))
  {
    ROS_WARN_STREAM("No SPI channel is defined. Will assume a single channel " << default_spi_channel_);
    spi_channel_.push_back(default_spi_channel_);
  }
  else
  {
    n.getParam("SPI_sensor_channel", spi_channel_);
  }
  for (std::vector<int>::const_iterator channel_iter = spi_channel_.begin(); channel_iter != spi_channel_.end();
        ++channel_iter)
  {
    if (*channel_iter < 0 || *channel_iter > NUM_SPI_OUTPUTS)
    {
      ROS_FATAL("spi channel parameter should be larger than or equal 0 "
                "and smaller than number of SPI outputs");
      return false;
    }
    standard_commands_[*channel_iter].packet.num_bytes = sensor_message_length_;
    topic_name << "_" << *channel_iter;
  }

  sensor_data_publisher_.init(n, topic_name.str(), 1);
  publisher_counter_ = 0;
  first_run_ = true;

  return true;
}

void SPISensorReadController::update(const ros::Time& time, const ros::Duration& dur)
{
  if (first_run_)  // configuring the channel for communication
  {
    first_run_ = false;
    spi_->command_->command_type = static_cast<int16u>(1);

    // setting up spi
    for (std::vector<int>::const_iterator channel_iter = spi_channel_.begin(); channel_iter != spi_channel_.end();
          ++channel_iter)
    {
      standard_commands_[*channel_iter].packet.SPI_config = 1;
      standard_commands_[*channel_iter].packet.clock_divider = static_cast<int16u>(16);
      standard_commands_[*channel_iter].packet.SPI_config |= static_cast<int16u>(0);
      standard_commands_[*channel_iter].packet.SPI_config |= 0;
      standard_commands_[*channel_iter].packet.SPI_config |= 0;
      standard_commands_[*channel_iter].packet.inter_byte_gap = 0;
      cmd_pin_output_states_pre_ = 0;
      cmd_pin_output_states_post_ |= chip_select_masks_[*channel_iter];
    }
  }
  // the command will be sent at the end of the iteration,
  // removing the command from the queue but not freeing the
  // memory yet
  sensor_msg_.data.resize(spi_channel_.size());
  for (std::vector<int>::const_iterator channel_iter = spi_channel_.begin(); channel_iter != spi_channel_.end();
        ++channel_iter)
  {
    if (status_queue_[*channel_iter].size() > 0)
    {
      if (status_queue_[*channel_iter].front().second == NULL)
      {
        if (new_command)
        {
          new_command = false;
          spi_->nullify_command(*channel_iter);
        }

        // the response has not been received. If the command type is NORMAL
        // then the response can be updated (it's INVALID until the SPI responds)
        if (spi_->state_->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL)
        {
          status_queue_[*channel_iter].front().second =
                  new SPI_PACKET_IN(spi_->state_->info_type.status_data.spi_in[*channel_iter]);
          unsigned int high_byte =
              static_cast<unsigned int>(status_queue_[*channel_iter].front().second->data_bytes[0] & 0x3F);
          unsigned int low_byte =
              static_cast<unsigned int>(status_queue_[*channel_iter].front().second->data_bytes[1]);
          ROS_DEBUG_STREAM("sensor value is " << (high_byte << 8 | low_byte));
          // channel_iter - spi_channel_.begin() is the index of channel_iter in vector
          sensor_msg_.data[channel_iter - spi_channel_.begin()] =
              (static_cast<double>((high_byte << 8 | low_byte) *360) / 16384);
        }
      }
      status_queue_[*channel_iter].pop();
    }
    try
    {
      standard_commands_[*channel_iter].packet.data_bytes[0] = 0xFF;
      standard_commands_[*channel_iter].packet.data_bytes[1] = 0xFF;
      command_queue_[*channel_iter].push(&standard_commands_[*channel_iter]);
    }
    catch(...)
    {
      ROS_ERROR_STREAM("something bad happened");
    }
    try
    {
      status_queue_[*channel_iter].push(std::pair<SplittedSPICommand*, SPI_PACKET_IN*>());
      status_queue_[*channel_iter].front().first = command_queue_[*channel_iter].front();

      // now we copy the command to the hardware interface
      copy_splitted_to_cmd_(*channel_iter);

      new_command = true;
    }
    catch(...)
    {
      ROS_ERROR_STREAM("error while copy_splitted_to_cmd_");
    }
    command_queue_[*channel_iter].pop();
  }

  if (publisher_counter_ % 10 == 0)
  {
    if (sensor_data_publisher_.trylock())
    {
      sensor_data_publisher_.msg_ = sensor_msg_;
      sensor_data_publisher_.unlockAndPublish();
    }
  }
  publisher_counter_++;
}
std::vector<double> SPISensorReadController::get_sensor_value()
{
  return sensor_msg_.data;
}

std::vector<int> SPISensorReadController::get_spi_channel()
{
  return spi_channel_;
}
} /* namespace ronex */
