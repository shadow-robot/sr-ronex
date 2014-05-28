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
 * @file   standard_ethercat_device.h
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Mon Jun 25 11:36:54 2012
 * @brief  Generic driver for a standard EtherCAT Slave.
 *         Reads information about inputs and outputs from the SII (in eeprom memory) 
 *         of the slave Drivers for every kind of standard ethercat device 
 *         should inherit from this class.
 **/

#ifndef STANDARD_ETHERCAT_DEVICE_H
#define STANDARD_ETHERCAT_DEVICE_H

#include <ros_ethercat_hardware/ethercat_hardware.h>
#include "realtime_tools/realtime_publisher.h"
#include "sr_ronex_drivers/cod_decod/cod_decod_manager.hpp"


class StandardEthercatDevice : public EthercatDevice
{
public:
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

protected:
  string reason_;
  int level_;
  int device_offset_;      //!< Offset of device position from first device

protected:
  int command_base_;
  int status_base_;

  int n_digital_outputs;
  int n_analog_outputs;
  int n_digital_inputs;
  int n_analog_inputs;
  int n_PWM_outputs;

  boost::scoped_ptr<sr_cod_decod::CodDecodManager> cod_decod_manager_;

};

#endif /* STANDARD_ETHERCAT_DEVICE_H */

