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
 * @file   sr_tcat.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX TCAT module.
 **/

#include <sr_ronex_drivers/sr_tcat.hpp>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <math.h>

#include "sr_ronex_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrTCAT, EthercatDevice);

const std::string SrTCAT::product_alias_ = "tcat";

SrTCAT::SrTCAT() :
  node_("~"), previous_sequence_number_(0)
{}

SrTCAT::~SrTCAT()
{
  //remove parameters from server
  std::stringstream param_path;
  param_path << "/ronex/devices/" << parameter_id_ ;
  ros::param::del(param_path.str());
}

void SrTCAT::construct(EtherCAT_SlaveHandler *sh, int &start_address)
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

  ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);

  ROS_INFO("Finished constructing the SrTCAT driver");
}

int SrTCAT::initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();// - hand_->getBridgeRingPosition();

  build_topics_();

  ROS_INFO_STREAM("Adding a "<< product_alias_ <<" RoNeX module to the hardware interface: " << device_name_);
  //Using the name of the ronex to prefix the state topic

  return 0;
}

void SrTCAT::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_02000003* command = (RONEX_COMMAND_02000003*)(buffer);

  command->command_type = RONEX_COMMAND_02000003_COMMAND_TYPE_NORMAL;
}

bool SrTCAT::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_02000003* status_data = (RONEX_STATUS_02000003 *)(this_buffer+  command_size_);
  u_int64_t timestamp_64;

  // Checking that the received command type matches RONEX_COMMAND_02000003_COMMAND_TYPE_NORMAL
  // (the one that was used in the command). The ronex firmware will answer with whatever command type we send.
  // The purpose of this is to filter those status_data structures that come filled with zeros due to the jitter
  // in the realtime loop. The jitter causes that the host tries to read the status when the microcontroller in the ronex
  // module has not finished writing it to memory yet.
  ROS_DEBUG_STREAM("-----\nNEW unpack");
  ROS_DEBUG_STREAM("   command type:    " << status_data->command_type << " ("<< RONEX_COMMAND_02000003_COMMAND_TYPE_NORMAL <<")");
  ROS_DEBUG_STREAM("   receiver number: " << status_data->receiver_number);
  ROS_DEBUG_STREAM("   seq number:      " << status_data->sequence_number << " ("<<previous_sequence_number_<<")");

  if( status_data->command_type == RONEX_COMMAND_02000003_COMMAND_TYPE_NORMAL)
  {
    //ignore if receiver_number = -1 (data is not filled in)
    if( status_data->receiver_number != -1 )
    {
      state_msg_.received_data[status_data->receiver_number].data_received = true;

      state_msg_.sequence_number = status_data->sequence_number;
      //fill in the state message with the new data.
      state_msg_.received_data[status_data->receiver_number].reserved.resize(NUM_RESERVED_WORDS);
      for(size_t i=0 ; i<NUM_RESERVED_WORDS; ++i)
        state_msg_.received_data[status_data->receiver_number].reserved[i] = status_data->receiver_data.reserved[i];

      state_msg_.received_data[status_data->receiver_number].impulse_response.resize(IMPULSE_RESPONSE_SIZE);
      for(size_t i=0 ; i<IMPULSE_RESPONSE_SIZE; ++i)
      {
        state_msg_.received_data[status_data->receiver_number].impulse_response[i].real = status_data->receiver_data.impulse_response[i].real;
        state_msg_.received_data[status_data->receiver_number].impulse_response[i].imaginary = status_data->receiver_data.impulse_response[i].imaginary;
      }

      state_msg_.received_data[status_data->receiver_number].first_sample_number = status_data->receiver_data.first_sample_number;

      state_msg_.received_data[status_data->receiver_number].payload.resize(PAYLOAD_MAX_SIZE);
      for(size_t i=0 ; i<PAYLOAD_MAX_SIZE; ++i)
        state_msg_.received_data[status_data->receiver_number].payload[i] = status_data->receiver_data.payload[i];

      state_msg_.received_data[status_data->receiver_number].rx_frame_information = status_data->receiver_data.rx_frame_information;
      state_msg_.received_data[status_data->receiver_number].std_noise = status_data->receiver_data.std_noise;
      state_msg_.received_data[status_data->receiver_number].flags = status_data->receiver_data.flags;
      state_msg_.received_data[status_data->receiver_number].FPI = FPI_FIXED_POINT_TO_FLOAT(status_data->receiver_data.FPI);

      timestamp_64   = status_data->receiver_data.timestamp_H;
      timestamp_64 <<= 32;
      timestamp_64  += status_data->receiver_data.timestamp_L;

      state_msg_.received_data[status_data->receiver_number].timestamp_ns = static_cast<double>(timestamp_64) * (15.65/1000.0);

      ROS_DEBUG_STREAM("   timestamp:       " << state_msg_.received_data[status_data->receiver_number].timestamp_ns);

      //printf("0x%04x%08x = %f\n", status_data->receiver_data.timestamp_H, status_data->receiver_data.timestamp_L, state_msg_.received_data[status_data->receiver_number].timestamp_ns);
      //state_msg_.received_data[status_data->receiver_number].timestamp_ns = static_cast<double>(status_data->receiver_data.timestamp_L + (static_cast<u_int64_t>(status_data->receiver_data.timestamp_H) << 32)*(15.65/1000.0));
    }
  }
  
  //publishing  if the sequence number is increased
  if ( status_data->sequence_number && 
      (status_data->sequence_number != previous_sequence_number_ )
     )
    {
      state_msg_.header.stamp = ros::Time::now();
      
      //publish the message
      if( state_publisher_->trylock() )
      {
        state_publisher_->msg_ = state_msg_;
        state_publisher_->unlockAndPublish();
      }
      
      //reset the data received flags to false
      for(size_t i=0; i<NUM_RECEIVERS; ++i)
        state_msg_.received_data[i].data_received = false;
      
      previous_sequence_number_ = status_data->sequence_number;
    }
  
  return true;
}

void SrTCAT::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  d.name = device_name_;
  d.summary(d.OK, "OK");
  d.hardware_id = serial_number_;

  d.clear();
}

void SrTCAT::build_topics_()
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
  state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_ronex_msgs::TCATState>(node_, device_name_ + "/state", 1));
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
