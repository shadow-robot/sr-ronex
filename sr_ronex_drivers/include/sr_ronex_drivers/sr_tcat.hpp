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
 * @file   sr_tcat.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX TCAT module.
 **/

#ifndef _SR_TCAT_HPP_
#define _SR_TCAT_HPP_

#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_msgs/TCATState.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000003_TCAT_00.h>

#include <boost/ptr_container/ptr_vector.hpp>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/GeneralIOConfig.h"

using namespace std;

class SrTCAT : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrTCAT();
  virtual ~SrTCAT();

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

  /**
   * Stores the previous sequence number: we publish the data when this
   * number is changed as it means we've received all the data
   */
  int16u previous_sequence_number_;

  ///Name under which the RoNeX will appear (prefix the topics etc...)
  std::string device_name_;
  std::string serial_number_;

  ///Offset of device position from first device
  int device_offset_;

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  ///publisher for the data.
  boost::shared_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::TCATState> > state_publisher_;
  ///Temporary message
  sr_ronex_msgs::TCATState state_msg_;

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

#endif /* _SR_TCAT_HPP_ */
