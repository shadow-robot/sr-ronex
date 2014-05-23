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
 * @file   sr_board_mk2_gio.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX mk2 General I/O module.
 **/

#ifndef _SR_BOARD_MK2_GIO_HPP_
#define _SR_BOARD_MK2_GIO_HPP_

#include <ros_ethercat_hardware/ethercat_device.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_msgs/GeneralIOState.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000001_GIO_00.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/GeneralIOConfig.h"

using namespace std;

class SrBoardMk2GIO : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrBoardMk2GIO();
  virtual ~SrBoardMk2GIO();

  void dynamic_reconfigure_cb(sr_ronex_drivers::GeneralIOConfig &config, uint32_t level);

protected:
  ///Replaces the product ID with a human readable product alias.
  static const std::string product_alias_;

  ///A unique identifier for the ronex (either serial number or alias if provided)
  std::string ronex_id_;

  string reason_;
  int level_;

  int command_base_;
  int status_base_;

  ros::NodeHandle node_;

  ///The GeneralIO module which is added as a CustomHW to the hardware interface
  ronex::GeneralIO *general_io_;

  /**
   * A counter used to publish the data at 100Hz:
   *  count 10 cycles, then reset the cycle_count to 0.
   */
  short cycle_count_;

  ///the digital commands sent at each cycle (updated when we call the topic)
  int32u digital_commands_;

  ///Name under which the RoNeX will appear (prefix the topics etc...)
  std::string device_name_;
  std::string serial_number_;

  ///Offset of device position from first device
  int device_offset_;

  ///True if a stacker board is plugged in the RoNeX
  bool has_stacker_;

  ///False to run digital pins as output, True to run as input
  std::vector<bool> input_mode_;

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  ///publisher for the data.
  boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::GeneralIOState> > state_publisher_;
  ///Temporary message
  sr_ronex_msgs::GeneralIOState state_msg_;

  ///Dynamic reconfigure server for setting the parameters of the driver
  boost::scoped_ptr<dynamic_reconfigure::Server<sr_ronex_drivers::GeneralIOConfig> > dynamic_reconfigure_server_;

  dynamic_reconfigure::Server<sr_ronex_drivers::GeneralIOConfig>::CallbackType function_cb_;

  ///building the topics for publishing the state.
  void build_topics_();

  ///Id of this ronex on the parameter server
  int parameter_id_;
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _SR_BOARD_MK2_GIO_HPP_ */

