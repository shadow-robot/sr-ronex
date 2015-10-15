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
 * @file   AUTOMATIC_GENERATOR_FILE_NAME.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX AUTOMATIC_GENERATOR_REPLACE_MODULE_NAME module.
 **/

#pragma once

#include <ros_ethercat_hardware/ethercat_device.h>
#include <realtime_tools/realtime_publisher.h>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <queue>
#include <string>

#include <dynamic_reconfigure/server.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0xAUTOMATIC_GENERATOR_REPLACE_PRODUCT_ID_AUTOMATIC_GENERATOR_REPLACE_MODULE_NAME_00.h>


class SrBoardAUTOMATIC_GENERATOR_REPLACE_MODULE_NAME : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed = true);

  SrBoardAUTOMATIC_GENERATOR_REPLACE_MODULE_NAME();
  virtual ~SrBoardAUTOMATIC_GENERATOR_REPLACE_MODULE_NAME();

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

  /**
   * A counter used to publish the data at 100Hz:
   *  count 10 cycles, then reset the cycle_count to 0.
   */
  int16_t cycle_count_;

  /// Name under which the RoNeX will appear (prefix the topics etc...)
  std::string device_name_;
  std::string serial_number_;

  /// Offset of device position from first device
  int device_offset_;

  /// True if a stacker board is plugged in the RoNeX
  bool has_stacker_;

  // Number of stacks present
  int stack;

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

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
