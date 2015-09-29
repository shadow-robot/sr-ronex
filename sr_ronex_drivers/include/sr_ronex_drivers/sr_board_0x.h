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
 * @file   sr_board_0x.h
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Mon Jun 25 11:36:54 2012
 * @brief  Generic driver for a standard EtherCAT Slave.
 *         Reads information about inputs and outputs from the SII 
 *         (in eeprom memory) of the slave Drivers for every kind 
 *         of device should inherit from this class.
 **/

#ifndef SR_BOARD_0X_H
#define SR_BOARD_0X_H

#include <sr_ronex_drivers/standard_ethercat_device.h>

#include <vector>
using namespace std;


class SrBoard0X : public StandardEthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);

protected:

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);
};

#endif /* SR_BOARD_0X_H */

