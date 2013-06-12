/**
 * @file   sr_board_mk2_gio.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 11 11:54:22 2013
 *
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
 * @brief Driver for the RoNeX mk2 General I/O module.
 *
 *
 */

#ifndef _SR_BOARD_MK2_GIO_HPP_
#define _SR_BOARD_MK2_GIO_HPP_

#include <ethercat_hardware/ethercat_device.h>
#include "realtime_tools/realtime_publisher.h"

#include <sr_ronex_external_protocol/Ronex_Protocol_0x0000000C_GIO_00.h>

#include <vector>
using namespace std;


class SrBoardMk2GIO : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrBoardMk2GIO();
  virtual ~SrBoardMk2GIO();

protected:
  string reason_;
  int level_;

  int command_base_;
  int status_base_;

  ///Offset of device position from first device
  int device_offset_;

  ///True if a stacker board is plugged in the RoNeX
  bool has_stacker_;

  size_t n_digital_outputs;
  size_t n_digital_inputs;
  size_t n_analog_outputs;
  size_t n_analog_inputs;
  size_t n_PWM_outputs;

  int writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length);
  int readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length);

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void multiDiagnostics(vector<diagnostic_msgs::DiagnosticStatus> &vec, unsigned char *buffer);
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _SR_BOARD_MK2_GIO_HPP_ */

