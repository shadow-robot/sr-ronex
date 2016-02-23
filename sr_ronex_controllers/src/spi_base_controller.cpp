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

#include "sr_ronex_controllers/spi_base_controller.hpp"
#include "pluginlib/class_list_macros.h"
#include <utility>
#include <string>

namespace ronex
{
SPIBaseController::SPIBaseController()
  : loop_count_(0), command_queue_(NUM_SPI_OUTPUTS), status_queue_(NUM_SPI_OUTPUTS), delete_status_(NUM_SPI_OUTPUTS)
{}

bool SPIBaseController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  return pre_init_(robot, n);
}

bool SPIBaseController::pre_init_(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;

  std::string ronex_id;
  if (!node_.getParam("ronex_id", ronex_id))
  {
    ROS_ERROR("No RoNeX ID given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  // get the path from the parameters
  std::string path;
  int parameter_id = get_ronex_param_id(ronex_id);
  {
    if ( parameter_id == -1 )
    {
      ROS_ERROR_STREAM("Could not find the RoNeX id in the parameter server: " << ronex_id <<
                               " not loading the controller.");
      return false;
    }
    else
    {
      std::stringstream ss;
      ss << "/ronex/devices/" << parameter_id << "/path";
      if ( !ros::param::get(ss.str(), path) )
      {
        ROS_ERROR_STREAM("Couldn't read the parameter " << ss.str() <<
                                 " from the parameter server. Not loading the controller.");
        return false;
      }
    }
  }
  topic_prefix_ = path;

  spi_ = static_cast<ronex::SPI*>(robot->getCustomHW(path));
  if (spi_ == NULL)
  {
    ROS_ERROR_STREAM("Could not find RoNeX module: " << ronex_id << " not loading the controller");
    return false;
  }

  // preallocating memory for the command and statues queues
  for (uint16_t spi_index = 0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
  {
    std::queue<int, boost::circular_buffer<SplittedSPICommand> > cq(boost::circular_buffer<SplittedSPICommand>(
        NUM_BUFFER_ELEMENTS));
    command_queue_[spi_index] = cq;
    std::queue<int, boost::circular_buffer<std::pair<SplittedSPICommand, SPIResponse > > > sq(boost::circular_buffer<
        std::pair<SplittedSPICommand, SPIResponse > >(NUM_BUFFER_ELEMENTS));
    status_queue_[spi_index] = sq;
  }
  return true;
}

void SPIBaseController::starting(const ros::Time&)
{
  // @TODO: implement starting
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
void SPIBaseController::update(const ros::Time&, const ros::Duration&)
{
  for (uint16_t spi_index = 0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
  {
/*    if (spi_index == 2)
    {
      SPI_PACKET_IN response = SPI_PACKET_IN(spi_->state_->info_type.status_data.spi_in[spi_index]);
      std::ostringstream hex;
      for (uint16_t data_index = 0; data_index < 3; ++data_index)
      {
        try
        {
          hex << static_cast<unsigned int>(response.data_bytes[data_index]);
        }
          catch(...)
        {
          ROS_ERROR_STREAM("Can't cast to uint.");
          hex << "bad_data";
        }
        hex << " ";

      }
      ROS_ERROR_STREAM(loop_count_ << " " << hex.str());
    }*/

    // Check if the status has been processed
    if (delete_status_[spi_index] and status_queue_[spi_index].size() > 0)
    {
      status_queue_[spi_index].pop();
      delete_status_[spi_index] = false;
    }

    // Check if we need to update a status
    if ( status_queue_[spi_index].size() > 0)
    {
      // ROS_ERROR_STREAM("Updating spi_index: "<< spi_index << " len=" << status_queue_[spi_index].s
      // ROS_ERROR_STREAM("loop_count_"<< loop_count_);
      // ROS_ERROR_STREAM("loop_number"<< status_queue_[spi_index].front().second.loop_number );
      if (loop_count_ == status_queue_[spi_index].front().second.loop_number + 2 )
      {
        ROS_ERROR_STREAM("loop number:"<< status_queue_[spi_index].back().second.loop_number);
//        if (new_command[spi_index])
//        {
//          ROS_ERROR_STREAM("new command true");
//          new_command[spi_index] = false;
//          spi_->nullify_command(spi_index);
//          continue;
//        }

        // the response has not been received. If the command type is NORMAL
        // then the response can be updated (it's INVALID until the SPI responds)
        if ( spi_->state_->command_type == RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL )
        {
          ROS_ERROR_STREAM("command normal");
          status_queue_[spi_index].back().second.received = true;
          status_queue_[spi_index].back().second.packet = SPI_PACKET_IN(spi_->state_->info_type.status_data.spi_in[spi_index]);

          SPI_PACKET_IN response = SPI_PACKET_IN(spi_->state_->info_type.status_data.spi_in[spi_index]);
          std::ostringstream hex;
          for (uint16_t data_index = 0; data_index < 3; ++data_index)
          {
            try
            {
              hex << static_cast<unsigned int>(response.data_bytes[data_index]);
            }
              catch(...)
            {
              ROS_ERROR_STREAM("Can't cast to uint.");
              hex << "bad_data";
            }
            hex << " ";

          }
          ROS_ERROR_STREAM(hex.str());
        }
      }
    }
    // if no available command then send the NULL command
    if ( command_queue_[spi_index].empty() )
    {
      //ROS_ERROR_STREAM("command empty");l");

      spi_->nullify_command(spi_index);
    }
    else
    {
      // sending the available command

      // first we add the pointer to the command onto the status queue - the status received flag is still false
      // as we haven't received the response yet.
      ROS_ERROR_STREAM("*******************sending available cmd loop_count_:" << loop_count_);
      status_queue_[spi_index].push(std::pair<SplittedSPICommand, SPIResponse>());
      status_queue_[spi_index].back().first = command_queue_[spi_index].front();
      status_queue_[spi_index].back().second.received = false;
      status_queue_[spi_index].back().second.loop_number = loop_count_;

      // now we copy the command to the hardware interface
      copy_splitted_to_cmd_(spi_index);

      // the command will be sent at the end of the iteration,
      // removing the command from the queue but not freeing the
      // memory yet
      ROS_ERROR_STREAM("pop cmd spi_index: " << spi_index);
      command_queue_[spi_index].pop();
    }
  }

  loop_count_++;
}

void SPIBaseController::copy_splitted_to_cmd_(uint16_t spi_index)
{
  // Mask to avoid setting the CS for the other SPI ports
  uint16_t bit_mask_CS = PIN_OUTPUT_STATE_CS_0 | PIN_OUTPUT_STATE_CS_1 | PIN_OUTPUT_STATE_CS_2 | PIN_OUTPUT_STATE_CS_3;
  uint16_t bit_mask_no_CS = ~bit_mask_CS;
  uint16_t bit_mask_one_CS_bit = PIN_OUTPUT_STATE_CS_0 << spi_index;

  // setting the pre / post pin states (for all the spi outputs)
  // First we leave the existing values for the CS bits
  spi_->command_->pin_output_states_pre &= bit_mask_CS;
  // then we set the values for all the non-CS bits
  spi_->command_->pin_output_states_pre |= (cmd_pin_output_states_pre_ & bit_mask_no_CS);
  // then we set the value for the CS bit corresponding to the current spi_index
  spi_->command_->pin_output_states_pre &= (~bit_mask_one_CS_bit);
  spi_->command_->pin_output_states_pre |= (cmd_pin_output_states_pre_ & bit_mask_one_CS_bit);

  // We do the same for the post-state
  // First we leave the existing values for the CS bits
  spi_->command_->pin_output_states_post &= bit_mask_CS;
  // then we set the values for all the non-CS bits
  spi_->command_->pin_output_states_post |= (cmd_pin_output_states_post_ & bit_mask_no_CS);
  // then we set the value for the CS bit corresponding to the current spi_index
  spi_->command_->pin_output_states_post &= (~bit_mask_one_CS_bit);
  spi_->command_->pin_output_states_post |= (cmd_pin_output_states_post_ & bit_mask_one_CS_bit);

  // copying the packet data
  spi_->command_->spi_out[spi_index].clock_divider = command_queue_[spi_index].front().packet.clock_divider;
  spi_->command_->spi_out[spi_index].SPI_config = command_queue_[spi_index].front().packet.SPI_config;
  spi_->command_->spi_out[spi_index].inter_byte_gap = command_queue_[spi_index].front().packet.inter_byte_gap;
  spi_->command_->spi_out[spi_index].num_bytes = command_queue_[spi_index].front().packet.num_bytes;

  for (size_t i = 0; i < SPI_TRANSACTION_MAX_SIZE; ++i)
    spi_->command_->spi_out[spi_index].data_bytes[i] = command_queue_[spi_index].front().packet.data_bytes[i];
}
}  // namespace ronex

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
