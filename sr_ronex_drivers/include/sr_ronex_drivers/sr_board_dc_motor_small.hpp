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
 * @file   sr_board_dc_motor_small.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX DC_MOTOR_SMALL module.
 **/

#pragma once

#include <ros_ethercat_hardware/ethercat_device.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <queue>
#include <string>
#include "sr_ronex_msgs/DCMotorState.h"
#include <dynamic_reconfigure/server.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000009_DC_Motor_Small_00.h>

#include "sr_ronex_hardware_interface/DC_motor_small_hardware_interface.hpp"


class SrBoardDC_MOTOR_SMALL : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed = true);

  SrBoardDC_MOTOR_SMALL();
  virtual ~SrBoardDC_MOTOR_SMALL();

protected:
  /// Replaces the product ID with a human readable product alias.
  static const std::string product_alias_;

  /// A unique identifier for the ronex (either serial number or alias if provided)
  std::string ronex_id_;

  string reason_;
  int level_;

  int command_base_;
  int status_base_;

  ros::NodeHandle node_;

  ronex::DCMotor *dc_motor_small_;
  /**
   * A counter used to publish the data at 100Hz:
   *  count 10 cycles, then reset the cycle_count to 0.
   */
  int16_t cycle_count_;

  /// the digital commands sent at each cycle (updated when we call the topic)
  int32u digital_commands_;
  /// Name under which the RoNeX will appear (prefix the topics etc...)
  std::string device_name_;
  std::string serial_number_;

  /// Offset of device position from first device
  int device_offset_;

  /// True if a stacker board is plugged in the RoNeX
  bool has_stacker_;

  // Number of stacks present
  int stack;

  std::vector<bool> input_mode_;

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  /// publisher for the data.
  boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::DCMotorState> > state_publisher_;
  /// Temporary message
  sr_ronex_msgs::DCMotorState state_msg_;
  /// building the topics for publishing the state.
  void build_topics_();

  /// Id of this ronex on the parameter server
  int parameter_id_;
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
