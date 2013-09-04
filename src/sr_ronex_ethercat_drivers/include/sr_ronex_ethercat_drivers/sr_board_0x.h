/**
 * @file   sr_board_0x.h
 * @author Toni Oliver <toni@shadowrobot.com>
 * @date   Mon Jun 25 11:36:54 2012
 *
 * Copyright 2012 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @brief Generic driver for a standard EtherCAT Slave.
 * Reads information about inputs and outputs from the SII (in eeprom memory) of the slave
 * Drivers for every kind of device should inherit from this class.
 *
 *
 */

#ifndef SR_BOARD_0X_H
#define SR_BOARD_0X_H

#include <sr_ronex_ethercat_drivers/standard_ethercat_device.h>

#include <vector>
using namespace std;


class SrBoard0X : public StandardEthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrBoard0X();
  virtual ~SrBoard0X();
protected:

  int writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length);
  int readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length);
  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

};

#endif /* SR_BOARD_0X_H */

