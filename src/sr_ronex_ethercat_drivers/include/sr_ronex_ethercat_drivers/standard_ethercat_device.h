/**
 * @file   standard_ethercat_device.h
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Mon Jun 25 11:36:54 2012
 *
 * Copyright 2013 Shadow Robot Company Ltd.
*
* This program is Proprietary software: you cannot redistribute it or modify it
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.
*
 *
 * @brief Generic driver for a standard EtherCAT Slave.
 * Reads information about inputs and outputs from the SII (in eeprom memory) of the slave
 * Drivers for every kind of standard ethercat device should inherit from this class.
 *
 *
 */

#ifndef STANDARD_ETHERCAT_DEVICE_H
#define STANDARD_ETHERCAT_DEVICE_H

#include <ethercat_hardware/ethercat_device.h>
#include "realtime_tools/realtime_publisher.h"
#include "sr_ronex_ethercat_drivers/cod_decod/cod_decod_manager.hpp"


class StandardEthercatDevice : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  StandardEthercatDevice();
  virtual ~StandardEthercatDevice();
protected:
  string reason_;
  int level_;

  int writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length);
  int readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length);

  int device_offset_;      //!< Offset of device position from first device

protected:
  int command_base_;
  int status_base_;

  int n_digital_outputs;
  int n_analog_outputs;
  int n_digital_inputs;
  int n_analog_inputs;
  int n_PWM_outputs;

  boost::shared_ptr<sr_cod_decod::CodDecodManager> cod_decod_manager_;

};

#endif /* STANDARD_ETHERCAT_DEVICE_H */

