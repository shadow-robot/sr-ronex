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
 * @file   sr_board_adc16.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Driver for the RoNeX ADC16 module.
 **/

#include <sr_ronex_drivers/sr_board_adc16.hpp>
#include <ros_ethercat_model/robot_state.hpp>
#include <ros_ethercat_hardware/ethercat_hardware.h>

#include <sstream>
#include <iomanip>
#include <boost/lexical_cast.hpp>
#include <math.h>

#include "sr_ronex_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrBoardADC16, EthercatDevice);

const std::string SrBoardADC16::product_alias_ = "adc16";
using boost::lexical_cast;


SrBoardADC16::SrBoardADC16() :
  node_("~"), cycle_count_(0), has_stacker_(false)
{}

SrBoardADC16::~SrBoardADC16()
{
  //remove parameters from server
  string device_id = "/ronex/devices/" + lexical_cast<string>(parameter_id_ );
  ros::param::del(device_id);

  string adc16_device_name = "/ronex/adc16/" + serial_number_;
  ros::param::del(adc16_device_name);

  string controller_name = "/ronex_" + serial_number_ + "_passthrough";
}

void SrBoardADC16::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  sh_ = sh;
  serial_number_ = ronex::get_serial_number( sh );

  //get the alias from the parameter server if it exists
  std::string path_to_alias, alias;
  path_to_alias = "/ronex/mapping/" + serial_number_;
  if( ros::param::get(path_to_alias, alias))
  {
    ronex_id_ = alias;
  }
  else
  {
    //no alias found, using the serial number directly.
    ronex_id_ = serial_number_ ;
  }

  device_name_ = ronex::build_name( product_alias_, ronex_id_ );

  command_base_  = start_address;
  command_size_  = COMMAND_ARRAY_SIZE_BYTES;

  start_address += command_size_;
  status_base_   = start_address;
  status_size_   = STATUS_ARRAY_SIZE_BYTES;

  start_address += status_size_;

  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the board
  //

  if ( (PROTOCOL_TYPE) == (EC_BUFFERED) )
  {
    ROS_INFO("Using EC_BUFFERED");
  }
  else if ( (PROTOCOL_TYPE) == (EC_QUEUED) )
  {
    ROS_INFO("Using EC_QUEUED");
  }

  ROS_INFO("First FMMU (command) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", command_base_, command_size_,
           static_cast<int>(COMMAND_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,            // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                   // Logical Start Bit
                                      0x07,                   // Logical End Bit
                                      COMMAND_ADDRESS,        // Physical Start Address   (in ET1200 address space?)
                                      0x00,                   // Physical Start Bit
                                      false,                   // Read Enable
                                      true,                    // Write Enable
                                      true                     // Channel Enable
    );


  // WARNING!!!
  // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode EC_QUEUED, the so called mailbox mode)

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the board
  //
  ROS_INFO("Second FMMU (status) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", status_base_, status_size_,
           static_cast<int>(STATUS_ADDRESS) );
  EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      STATUS_ADDRESS,
                                      0x00,
                                      true,
                                      false,
                                      true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

// SyncMan takes the physical address
  (*pd)[0] = EC_SyncMan(COMMAND_ADDRESS,              command_size_,    PROTOCOL_TYPE, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(STATUS_ADDRESS,               status_size_,     PROTOCOL_TYPE);


  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent    = true;

  (*pd)[1].ChannelEnable = true;

  sh->set_pd_config(pd);

  ROS_INFO("Finished constructing the SrBoardADC16 driver");
}

int SrBoardADC16::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  digital_commands_ = 0;
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();

  //add the RoNeX ADC16 module to the hw interface
  ros_ethercat_model::RobotState *robot_state = static_cast<ros_ethercat_model::RobotState*>(hw);
  robot_state->custom_hws_.insert(device_name_, new ronex::ADC16());
  adc16_ = static_cast<ronex::ADC16*>(robot_state->getCustomHW(device_name_));

  build_topics_();

  ROS_INFO_STREAM("Adding an ADC16 RoNeX module to the hardware interface: " << device_name_);
  //Using the name of the ronex to prefix the state topic
  feedback_flag_ = 0;

  return 0;
}

void SrBoardADC16::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_02000008* command = (RONEX_COMMAND_02000008*)(buffer);

  if (! config_received_)
  {
    command->command_type = RONEX_COMMAND_02000008_COMMAND_TYPE_GET_CONFIG_INFO;
  }
  else if (reg_flag_)
  {
    switch(reg_state_)
    {
      case 0:
        if (!command_queue_.empty())
        {
          command->command_type = RONEX_COMMAND_02000008_COMMAND_TYPE_SET_REG_VAL;
          command->address = command_queue_.front().address;
          for(int i = 0; i < 3; ++i )
          {
            command->values[i] = command_queue_.front().values[i];
          }
          feedback_flag_ += 1;
          command_queue_.pop();
        }
        else
        {
          command->command_type = RONEX_COMMAND_02000008_COMMAND_TYPE_WRITE_REGS;
          reg_state_ = 1;
          feedback_flag_ += 1;
        }
        break;
      case 1:
        command->command_type = RONEX_COMMAND_02000008_COMMAND_TYPE_NORMAL;
        reg_flag_ = false;
        break;
    }
  }

  else
  {
    command->command_type = RONEX_COMMAND_02000008_COMMAND_TYPE_NORMAL;
    //digital command
    feedback_flag_ = 0;
    for (size_t i = 0; i < adc16_->command_.digital_.size(); ++i)
    {
      if (input_mode_[i])

      {
        // Just set the pin to input mode, gets read in the status
        ronex::set_bit(digital_commands_, i*2, 1);
      }
      else
      { // Output
        ronex::set_bit(digital_commands_, i*2, 0);
        ronex::set_bit(digital_commands_, i*2+1, adc16_->command_.digital_[i]);
      }
    }
    command->pin_output_states = static_cast<int32u>(digital_commands_);
  }
}

bool SrBoardADC16::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_02000008* status_data = (RONEX_STATUS_02000008 *)(this_buffer+  command_size_);

  // Checking that the received command type matches RONEX_COMMAND_02000001_COMMAND_TYPE_NORMAL
  // (the one that was used in the command). The ronex firmware will answer with whatever command type we send.
  // The purpose of this is to filter those status_data structures that come filled with zeros due to the jitter
  // in the realtime loop. The jitter causes that the host tries to read the status when the microcontroller in the ronex
  // module has not finished writing it to memory yet.

  if(status_data->command_type == RONEX_COMMAND_02000008_COMMAND_TYPE_NORMAL)
  {
    for(size_t i = 0; i < adc16_->state_.analogue_.size(); ++i )
    {
      adc16_->state_.analogue_[i] = status_data->info_type.status_data.analogue_in[i];
    }

    for(size_t i = 0; i < adc16_->state_.digital_.size(); ++i )
    {
      adc16_->state_.digital_[i] = ronex::check_bit(status_data->info_type.status_data.pin_input_states_DIO, i);
    }

    int mask = 1;
    int count = 15;
    int bits_count = 0;

    //pin order comes back the reverse of values on the data sheet, so 7->0 for differential and 15->0 for single
    for (int j = 0; j < stack; j++)
    {
      unsigned short int differential = values_d_[j];
      unsigned short int single_ended = (values_s1_[j] << 8) | values_s0_[j];
      unsigned short int padded_single_ended = (padded_s1_[j] << 8) | padded_s0_[j];
      for (int n = 0; n < 8; n++)
      {
        if(differential & mask << n)
        {
          adc16_->state_.adc_[count] = status_data->info_type.status_data.adc16[bits_count].U16;
          adc16_->state_.adc_[count -1] = status_data->info_type.status_data.adc16[bits_count].U16;
          bits_count += 1;
        }
        count = count -2;
      }
      count = ((j + 1) * 16) - 1; //start at 15, 31 or 47
      for (int n = 0; n < 16; n++)
      {
        if(single_ended & mask << n)
        {
          adc16_->state_.adc_[count] = status_data->info_type.status_data.adc16[bits_count].U16;
          bits_count += 1;
        }
        else if(padded_single_ended & mask << n)
        {
          bits_count += 1;
        }
        count -=1;
      }
    }
  }
  else if(status_data->command_type == RONEX_COMMAND_02000008_COMMAND_TYPE_SET_REG_VAL)
  {
    adc16_->state_.address_ = status_data->info_type.register_feedback.address;

    // check the feedback address value against the flag number, if feedback incorrect, reset queue
    if(((feedback_flag_ == 2) & (adc16_->state_.address_ != 4)) | ((feedback_flag_ == 3) & (adc16_->state_.address_ != 5)) |
    ((feedback_flag_ == 4) & (adc16_->state_.address_ != 3)))
    {
      reg_state_ = 0;
      command_queue_ = queue_backup_;
    }
  }
  else if(status_data->command_type == RONEX_COMMAND_02000008_COMMAND_TYPE_GET_CONFIG_INFO)
  {
    if( adc16_->state_.analogue_.empty())
    {
      size_t nb_adc_pub;
      //The publishers haven't been initialised yet.
      // Checking if the stacker board is plugged in or not
      // to determine the number of publishers.
      if (status_data->info_type.config_info.flags & RONEX_02000008_FLAGS_STACKER_0_PRESENT)
      {
        has_stacker_ = true;
        nb_adc_pub = NUM_ADC16_INPUTS / 3;
      }
      if (status_data->info_type.config_info.flags & RONEX_02000008_FLAGS_STACKER_1_PRESENT)
      {
        has_stacker_ = true;
        nb_adc_pub = (NUM_ADC16_INPUTS * 2) / 3;
      }
      if (status_data->info_type.config_info.flags & RONEX_02000008_FLAGS_STACKER_2_PRESENT)
      {
        has_stacker_ = true;
        nb_adc_pub = NUM_ADC16_INPUTS;
      }

      //resizing the ADC in the HardwareInterface
      adc16_->state_.analogue_.resize(NUM_ANALOGUE_INPUTS);
      adc16_->state_.digital_.resize(NUM_DIGITAL_IO);
      adc16_->state_.adc_.resize(nb_adc_pub);
      adc16_->command_.digital_.resize(NUM_DIGITAL_IO);

      input_mode_.assign(NUM_DIGITAL_IO, true);
      pin_mode_.assign(nb_adc_pub, 0);

      //init the state message
      state_msg_.analogue.resize(NUM_ANALOGUE_INPUTS);
      state_msg_.digital.resize(NUM_DIGITAL_IO);
      state_msg_.adc.resize(nb_adc_pub);
      state_msg_.input_mode.resize(NUM_DIGITAL_IO);

      //dynamic reconfigure server is instantiated here
      // as we need the different vectors to be initialised
      // before running the first configuration.
      dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sr_ronex_drivers::ADC16Config>(ros::NodeHandle(device_name_)));
      function_cb_ = boost::bind(&SrBoardADC16::dynamic_reconfigure_cb, this, _1, _2);
      dynamic_reconfigure_server_->setCallback(function_cb_);
    } //end first time, the sizes are properly initialised, simply fill in the data

    config_received_ = true;
  }

  adc16_->state_.command_type_ = status_data->command_type;

  //publishing at 100Hz
  if(cycle_count_ > 9)
  {
    state_msg_.header.stamp = ros::Time::now();

    //update state message
    for(size_t i=0; i < adc16_->state_.analogue_.size(); ++i)
    {
      state_msg_.analogue[i] = adc16_->state_.analogue_[i];
    }

    for(size_t i=0; i < adc16_->state_.digital_.size(); ++i)
    {
      state_msg_.digital[i] = adc16_->state_.digital_[i];
      state_msg_.input_mode[i] = input_mode_[i];
    }
    for(size_t i=0; i < adc16_->state_.adc_.size(); ++i)
    {
      state_msg_.adc[i] = adc16_->state_.adc_[i];
    }

    state_msg_.command_type = adc16_->state_.command_type_;

    //publish
    if( state_publisher_->trylock() )
    {
      state_publisher_->msg_ = state_msg_;
      state_publisher_->unlockAndPublish();
    }

    cycle_count_ = 0;
  }

  cycle_count_++;
  return true;
}

void SrBoardADC16::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  d.name = device_name_;
  d.summary(d.OK, "OK");
  d.hardware_id = serial_number_;

  d.clear();
  if(has_stacker_)
    d.addf("Stacker Board", "True");
  else
    d.addf("Stacker Board", "False");
}

void SrBoardADC16::dynamic_reconfigure_cb(sr_ronex_drivers::ADC16Config &config, uint32_t level)
{
  //not very pretty but I couldnt think of an easy way to set them up
  // (dynamic reconfigure doesn't seem to support arrays)
  if(adc16_->command_.digital_.size() > 0)
    input_mode_[0] = config.input_mode_0;
  if(adc16_->command_.digital_.size() > 1)
    input_mode_[1] = config.input_mode_1;
  if(adc16_->command_.digital_.size() > 2)
    input_mode_[2] = config.input_mode_2;
  if(adc16_->command_.digital_.size() > 3)
    input_mode_[3] = config.input_mode_3;
  if(adc16_->command_.digital_.size() > 4)
    input_mode_[4] = config.input_mode_4;
  if(adc16_->command_.digital_.size() > 5)
    input_mode_[5] = config.input_mode_5;

  stack = 0;

  //pins are paired for 1 mode of either differential or single ended configuration
  if(adc16_->state_.adc_.size() >= 16)
  {
    pin_mode_[0] = config.pins_0_1;
    pin_mode_[1] = config.pins_2_3;
    pin_mode_[2] = config.pins_4_5;
    pin_mode_[3] = config.pins_6_7;
    pin_mode_[4] = config.pins_8_9;
    pin_mode_[5] = config.pins_10_11;
    pin_mode_[6] = config.pins_12_13;
    pin_mode_[7] = config.pins_14_15;
    stack = 1;
  }
  if(adc16_->state_.adc_.size() >= 32)
  {
    pin_mode_[8] = config.pins_16_17;
    pin_mode_[9] = config.pins_18_19;
    pin_mode_[10] = config.pins_20_21;
    pin_mode_[11] = config.pins_22_23;
    pin_mode_[12] = config.pins_24_25;
    pin_mode_[13] = config.pins_26_27;
    pin_mode_[14] = config.pins_28_29;
    pin_mode_[15] = config.pins_30_31;
    stack = 2;
  }
  if(adc16_->state_.adc_.size() == 48)
  {
    pin_mode_[16] = config.pins_32_33;
    pin_mode_[17] = config.pins_34_35;
    pin_mode_[18] = config.pins_36_37;
    pin_mode_[19] = config.pins_38_39;
    pin_mode_[20] = config.pins_40_41;
    pin_mode_[21] = config.pins_42_43;
    pin_mode_[22] = config.pins_44_45;
    pin_mode_[23] = config.pins_46_47;
    stack = 3;
  }

  //set values for each of the 3 registers,
  values_s0_.assign(3, 0);
  values_s1_.assign(3, 0);
  values_d_.assign(3, 0);
  //values to pad out the bits
  fake_values_s0_.assign(3, 0);
  fake_values_s1_.assign(3, 0);
  //value that is sent, sum of user requested and padding
  padded_s0_.assign(3, 0);
  padded_s1_.assign(3, 0);

  int bit = 1;
  int pin_count = (adc16_->state_.adc_.size()/(stack*2)) -1;

  //set values for s1 and d, first set of 8 pins. NOTE the pin order is in the reverse order
  //given on the data sheet for the chip 15->0 rather than 0->15

  for (int i = 0; i < stack; i++) //loop for each stack
  {
    for (signed int pin = 0; pin <= pin_count; pin++) //first 8 pins
    {
      if (pin <= pin_count - 4) //4 pairs make up the 8 pins
      {
        if (pin_mode_[pin] == 0) //no input
        {
          values_s1_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s1_[i] <<=2;
          fake_values_s1_[i] |= bit * 3;
        }
        else if (pin_mode_[pin] == 1) //single ended ADC input
        {
          values_s1_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s1_[i] <<=2;
          values_s1_[i] |= bit * 3;
        }
        else //differential ADC input
        {
          values_s1_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s1_[i] <<=1;
          values_d_[i] |= bit;
          fake_values_s1_[i] |= bit;
        }
      }

      //set values for s0 and d, second set of 8 pins
      else if (pin <= pin_count)
      {
        if (pin_mode_[pin] == 0) //no input
        {
          values_s0_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s0_[i] <<=2;
          fake_values_s0_[i] |= bit * 3;
        }
        else if (pin_mode_[pin] == 1) //single ended ADC input
        {
          values_s0_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s0_[i] <<=2;
          values_s0_[i] |= bit * 3;
        }
        else //differential ADC input
        {
          values_s0_[i] <<=2;
          values_d_[i] <<=1;
          fake_values_s0_[i] <<=1;
          values_d_[i] |= bit;
          fake_values_s0_[i] |= bit;
        }
      }
    }
    pin_count = pin_count + 8;
    padded_s0_[i] = values_s0_[i] | fake_values_s0_[i];
    padded_s1_[i] = values_s1_[i] | fake_values_s1_[i];
  }

  // set register values
  reg_flag_ = true;
  reg_state_ = 0;

  RONEX_COMMAND_02000008 command;

  // queue commands for the 3 registers
  command.address = RONEX_02000008_ADS1158_REGISTER_MUXSG0;
  command.values[0] = padded_s0_[0];
  command.values[1] = padded_s0_[1];
  command.values[2] = padded_s0_[2];

  command_queue_.push(command);

  command.address = RONEX_02000008_ADS1158_REGISTER_MUXSG1;
  command.values[0] = padded_s1_[0];
  command.values[1] = padded_s1_[1];
  command.values[2] = padded_s1_[2];

  command_queue_.push(command);

  command.address = RONEX_02000008_ADS1158_REGISTER_MUXDIF;
  command.values[0] = values_d_[0];
  command.values[1] = values_d_[1];
  command.values[2] = values_d_[2];

  //send last register values twice to catch all feedback (delayed by 1)
  command_queue_.push(command);
  command_queue_.push(command);

  // create backup to reset queue if any packets lost
  queue_backup_ = command_queue_;
}


void SrBoardADC16::build_topics_()
{
  //loading everything into the parameter server
  parameter_id_ = ronex::get_ronex_param_id("");
  std::stringstream param_path, tmp_param;
  param_path << "/ronex/devices/" << parameter_id_ << "/";
  tmp_param << ronex::get_product_code(sh_);
  ros::param::set(param_path.str() + "product_id", tmp_param.str());
  ros::param::set(param_path.str() + "product_name", product_alias_);
  ros::param::set(param_path.str() + "ronex_id", ronex_id_);

  //the device is stored using path as the key in the CustomHW map
  ros::param::set(param_path.str() + "path", device_name_);
  ros::param::set(param_path.str() + "serial", serial_number_);

  //Advertising the realtime state publisher
  state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_ronex_msgs::ADC16State>(node_, device_name_ + "/state", 1));
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
