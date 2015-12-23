/*
 * Copyright (c) 2015, Shadow Robot Company, All rights reserved.
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
 * @file   sr_board_dc_motor_small.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Driver for the RoNeX DC_MOTOR_SMALL module.
 **/

#include <sr_ronex_drivers/sr_board_dc_motor_small.hpp>
#include <ros_ethercat_model/robot_state.hpp>
#include <ros_ethercat_hardware/ethercat_hardware.h>

#include <sstream>
#include <iomanip>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <string>

#include "sr_ronex_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrBoardDC_MOTOR_SMALL, EthercatDevice);

const std::string SrBoardDC_MOTOR_SMALL::product_alias_ = "DC_MOTOR_SMALL";
using boost::lexical_cast;


SrBoardDC_MOTOR_SMALL::SrBoardDC_MOTOR_SMALL() :
  node_("~"), cycle_count_(0), has_stacker_(false)
{}

SrBoardDC_MOTOR_SMALL::~SrBoardDC_MOTOR_SMALL()
{
  // remove parameters from server
  string device_id = "/ronex/devices/" + lexical_cast<string>(parameter_id_);
  ros::param::del(device_id);

  string DC_MOTOR_SMALL_device_name = "/ronex/DC_MOTOR_SMALL/" + serial_number_;
  ros::param::del(DC_MOTOR_SMALL_device_name);

  string controller_name = "/ronex_" + serial_number_ + "_passthrough";
}

void SrBoardDC_MOTOR_SMALL::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  sh_ = sh;
  serial_number_ = ronex::get_serial_number(sh);

  // get the alias from the parameter server if it exists
  std::string path_to_alias, alias;
  path_to_alias = "/ronex/mapping/" + serial_number_;
  if ( ros::param::get(path_to_alias, alias))
  {
    ronex_id_ = alias;
  }
  else
  {
    // no alias found, using the serial number directly.
    ronex_id_ = serial_number_;
  }

  device_name_ = ronex::build_name(product_alias_, ronex_id_);

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

  ROS_INFO("First FMMU (command) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", command_base_,
           command_size_, static_cast<int>(COMMAND_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,            // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                     // Logical Start Bit
                                      0x07,                     // Logical End Bit
                                      COMMAND_ADDRESS,          // Physical Start Address   (in ET1200 address space?)
                                      0x00,                     // Physical Start Bit
                                      false,                    // Read Enable
                                      true,                     // Write Enable
                                      true);                    // Channel Enable



  // WARNING!!!
  // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode
  // EC_QUEUED, the so called mailbox mode)

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the board
  //
  ROS_INFO("Second FMMU (status) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", status_base_,
           status_size_, static_cast<int>(STATUS_ADDRESS) );
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

  ROS_INFO("Finished constructing the SrBoardDC_MOTOR_SMALL driver");
}

int SrBoardDC_MOTOR_SMALL::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  digital_commands_ = 0;
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();

  // add the RoNeX DC_MOTOR_SMALL module to the hw interface
  ros_ethercat_model::RobotState *robot_state = static_cast<ros_ethercat_model::RobotState*>(hw);

  build_topics_();

  ROS_INFO_STREAM("Adding an DC_MOTOR_SMALL RoNeX module to the hardware interface: " << device_name_);
  // Using the name of the ronex to prefix the state topic
  feedback_flag_ = 0;

  return 0;
}

void SrBoardDC_MOTOR_SMALL::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_02000009* command = reinterpret_cast<RONEX_COMMAND_02000009*>(buffer);
}

bool SrBoardDC_MOTOR_SMALL::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_02000009* status_data = reinterpret_cast<RONEX_STATUS_02000009 *>(this_buffer+  command_size_);

  // publishing at 100Hz
  if (cycle_count_ > 9)
  {
    cycle_count_ = 0;
  }

  cycle_count_++;
  return true;
}

void SrBoardDC_MOTOR_SMALL::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  d.name = device_name_;
  d.summary(d.OK, "OK");
  d.hardware_id = serial_number_;

  d.clear();
  if (has_stacker_)
    d.addf("Stacker Board", "True");
  else
    d.addf("Stacker Board", "False");
}

void SrBoardDC_MOTOR_SMALL::dynamic_reconfigure_cb(sr_ronex_drivers::DC_MOTOR_SMALLConfig &config, uint32_t level)
{
}


void SrBoardDC_MOTOR_SMALL::build_topics_()
{
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
