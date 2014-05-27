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
 * @file   sr_spi.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX SPI module.
 **/

#ifndef _SR_SPI_HPP_
#define _SR_SPI_HPP_

#include <ros_ethercat_hardware/ethercat_device.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_msgs/SPIState.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000002_SPI_00.h>
#include <sr_ronex_hardware_interface/spi_hardware_interface.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>

class SrSPI : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrSPI();
  virtual ~SrSPI();

protected:
  ///Replaces the product ID with a human readable product alias.
  static const std::string product_alias_;

  ///A unique identifier for the ronex (either serial number or alias if provided)
  std::string ronex_id_;

  std::string reason_;
  int level_;

  int command_base_;
  int status_base_;

  ros::NodeHandle node_;

  ///The SPI module which is added as a CustomHW to the hardware interface
  ronex::SPI* spi_;

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

  ///False to run digital pins as output, True to run as input
  std::vector<bool> input_mode_;

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  ///publisher for the data.
  boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::SPIState> > state_publisher_;
  ///Temporary message
  sr_ronex_msgs::SPIState state_msg_;

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

#endif /* _SR_SPI_HPP_ */
