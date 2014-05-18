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
 * @file   sr_board_mk2_gio.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Driver for the RoNeX mk2 General I/O module.
 **/

#include <sr_ronex_drivers/sr_board_mk2_gio.hpp>
#include <ros_ethercat_model/robot_state.hpp>
#include <ros_ethercat_hardware/ethercat_hardware.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>

#include "sr_ronex_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrBoardMk2GIO, EthercatDevice);

const std::string SrBoardMk2GIO::product_alias_ = "general_io";
using boost::lexical_cast;


SrBoardMk2GIO::SrBoardMk2GIO() :
  node_("~"), cycle_count_(0), has_stacker_(false)
{}

SrBoardMk2GIO::~SrBoardMk2GIO()
{
  //remove parameters from server
  string device_id = "/ronex/devices/" + lexical_cast<string>(parameter_id_ );
  ros::param::del(device_id);

  string general_io_name = "/ronex/general_io/" + serial_number_ + "/";
  ros::param::del(general_io_name + "pwm_clock_divider");
  for (size_t i = 0; i < general_io_->command_.digital_.size(); ++i)
    ros::param::del(general_io_name + "input_mode_" + lexical_cast<string>(i));
  for (size_t i = 0; i < general_io_->command_.pwm_.size(); ++i)
    ros::param::del(general_io_name + "pwm_period_" + lexical_cast<string>(i));

  string controller_name = "/ronex_" + serial_number_ + "_passthrough/";
  ros::param::del(controller_name + "ronex_id");
  ros::param::del(controller_name + "type");
}

void SrBoardMk2GIO::construct(EtherCAT_SlaveHandler *sh, int &start_address)
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

  ROS_INFO("Finished constructing the SrBoardMk2GIO driver");
}

int SrBoardMk2GIO::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  digital_commands_ = 0;
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();

  //add the RoNeX to the hw interface
  ros_ethercat_model::RobotState *robot_state = static_cast<ros_ethercat_model::RobotState*>(hw);
  robot_state->custom_hws_.insert(device_name_, new ronex::GeneralIO());
  general_io_ = static_cast<ronex::GeneralIO*>(robot_state->getCustomHW(device_name_));

  build_topics_();

  ROS_INFO_STREAM("Adding a general_io RoNeX module to the hardware interface: " << device_name_);
  //Using the name of the ronex to prefix the state topic

  return 0;
}

void SrBoardMk2GIO::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_02000001* command = (RONEX_COMMAND_02000001*)(buffer);

  command->command_type = RONEX_COMMAND_02000001_COMMAND_TYPE_NORMAL;

  //digital command
  for (size_t i = 0; i < general_io_->command_.digital_.size(); ++i)
  {
    if (input_mode_[i])
    {
      // Just set the pin to input mode, gets read in the status
      ronex::set_bit(digital_commands_, i*2, 1);
    }
    else
    { // Output
      ronex::set_bit(digital_commands_, i*2, 0);
      ronex::set_bit(digital_commands_, i*2+1, general_io_->command_.digital_[i]);
    }
  }

  command->digital_out = static_cast<int32u>(digital_commands_);

  //PWM command
  for (size_t i = 0; i < general_io_->command_.pwm_.size(); ++i)
  {
    command->pwm_module[i].pwm_period = general_io_->command_.pwm_[i].period;
    command->pwm_module[i].pwm_on_time_0 = general_io_->command_.pwm_[i].on_time_0;
    command->pwm_module[i].pwm_on_time_1 = general_io_->command_.pwm_[i].on_time_1;
  }

  command->pwm_clock_divider = general_io_->command_.pwm_clock_divider_;
}

bool SrBoardMk2GIO::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_02000001* status_data = (RONEX_STATUS_02000001 *)(this_buffer+  command_size_);

  // Checking that the received command type matches RONEX_COMMAND_02000001_COMMAND_TYPE_NORMAL
  // (the one that was used in the command). The ronex firmware will answer with whatever command type we send.
  // The purpose of this is to filter those status_data structures that come filled with zeros due to the jitter
  // in the realtime loop. The jitter causes that the host tries to read the status when the microcontroller in the ronex
  // module has not finished writing it to memory yet.
  if( status_data->command_type == RONEX_COMMAND_02000001_COMMAND_TYPE_NORMAL)
  {
    if( general_io_->state_.analogue_.empty())
    {
      size_t nb_analogue_pub, nb_digital_io, nb_pwm_modules;
      //The publishers haven't been initialised yet.
      // Checking if the stacker board is plugged in or not
      // to determine the number of publishers.
      if (status_data->flags & RONEX_02000001_FLAGS_STACKER_0_PRESENT)
      {
        has_stacker_ = true;
        nb_analogue_pub = NUM_ANALOGUE_INPUTS;
        nb_digital_io = NUM_DIGITAL_IO;
        nb_pwm_modules = NUM_PWM_MODULES;
      }
      else
      {
        has_stacker_ = false;
        nb_analogue_pub = NUM_ANALOGUE_INPUTS / 2;
        nb_digital_io = NUM_DIGITAL_IO / 2;
        nb_pwm_modules = NUM_PWM_MODULES / 2;
      }

      //resizing the GeneralIO in the HardwareInterface
      general_io_->state_.analogue_.resize(nb_analogue_pub);
      general_io_->state_.digital_.resize(nb_digital_io);
      general_io_->command_.digital_.resize(nb_digital_io);
      general_io_->command_.pwm_.resize(nb_pwm_modules);

      input_mode_.assign(nb_digital_io, true);

      //init the state message
      state_msg_.analogue.resize(nb_analogue_pub);
      state_msg_.digital.resize(nb_digital_io);
      state_msg_.input_mode.resize(nb_digital_io);

      //dynamic reconfigure server is instantiated here
      // as we need the different vectors to be initialised
      // before running the first configuration.
      dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sr_ronex_drivers::GeneralIOConfig>(ros::NodeHandle(device_name_)));
      function_cb_ = boost::bind(&SrBoardMk2GIO::dynamic_reconfigure_cb, this, _1, _2);
      dynamic_reconfigure_server_->setCallback(function_cb_);
    } //end first time, the sizes are properly initialised, simply fill in the data

    for(size_t i = 0; i < general_io_->state_.analogue_.size(); ++i )
    {
      general_io_->state_.analogue_[i] = status_data->analogue_in[i];
    }

    for(size_t i = 0; i < general_io_->state_.digital_.size(); ++i )
    {
      general_io_->state_.digital_[i] = ronex::check_bit(status_data->digital_in, i);
    }
  }

  //publishing at 100Hz
  if(cycle_count_ > 9)
  {
    state_msg_.header.stamp = ros::Time::now();

    //update state message
    for(size_t i=0; i < general_io_->state_.analogue_.size(); ++i)
    {
      state_msg_.analogue[i] = general_io_->state_.analogue_[i];
    }

    for(size_t i=0; i < general_io_->state_.digital_.size(); ++i)
    {
      state_msg_.digital[i] = general_io_->state_.digital_[i];
      state_msg_.input_mode[i] = input_mode_[i];
    }

    state_msg_.pwm_clock_divider = general_io_->command_.pwm_clock_divider_;

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

void SrBoardMk2GIO::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
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


void SrBoardMk2GIO::dynamic_reconfigure_cb(sr_ronex_drivers::GeneralIOConfig &config, uint32_t level)
{
  general_io_->command_.pwm_clock_divider_ = static_cast<int16u>(config.pwm_clock_divider);

  //not very pretty but I couldnt think of an easy way to set them up
  // (dynamic reconfigure doesn't seem to support arrays)
  if(general_io_->command_.digital_.size() > 0)
    input_mode_[0] = config.input_mode_0;
  if(general_io_->command_.digital_.size() > 1)
    input_mode_[1] = config.input_mode_1;
  if(general_io_->command_.digital_.size() > 2)
    input_mode_[2] = config.input_mode_2;
  if(general_io_->command_.digital_.size() > 3)
    input_mode_[3] = config.input_mode_3;
  if(general_io_->command_.digital_.size() > 4)
    input_mode_[4] = config.input_mode_4;
  if(general_io_->command_.digital_.size() > 5)
    input_mode_[5] = config.input_mode_5;
  if(general_io_->command_.digital_.size() > 6)
    input_mode_[6] = config.input_mode_6;
  if(general_io_->command_.digital_.size() > 7)
    input_mode_[7] = config.input_mode_7;
  if(general_io_->command_.digital_.size() > 8)
    input_mode_[8] = config.input_mode_8;
  if(general_io_->command_.digital_.size() > 9)
    input_mode_[9] = config.input_mode_9;
  if(general_io_->command_.digital_.size() > 10)
    input_mode_[10] = config.input_mode_10;
  if(general_io_->command_.digital_.size() > 11)
    input_mode_[11] = config.input_mode_11;

  if( general_io_->command_.pwm_.size() > 0 )
    general_io_->command_.pwm_[0].period = static_cast<int16u>(config.pwm_period_0);
  if( general_io_->command_.pwm_.size() > 1 )
    general_io_->command_.pwm_[1].period = static_cast<int16u>(config.pwm_period_1);
  if( general_io_->command_.pwm_.size() > 2 )
    general_io_->command_.pwm_[2].period = static_cast<int16u>(config.pwm_period_2);
  if( general_io_->command_.pwm_.size() > 3 )
    general_io_->command_.pwm_[3].period = static_cast<int16u>(config.pwm_period_3);
  if( general_io_->command_.pwm_.size() > 4 )
    general_io_->command_.pwm_[4].period = static_cast<int16u>(config.pwm_period_4);
  if( general_io_->command_.pwm_.size() > 5 )
    general_io_->command_.pwm_[5].period = static_cast<int16u>(config.pwm_period_5);
}

void SrBoardMk2GIO::build_topics_()
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
  state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_ronex_msgs::GeneralIOState>(node_, device_name_ + "/state", 1));
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
