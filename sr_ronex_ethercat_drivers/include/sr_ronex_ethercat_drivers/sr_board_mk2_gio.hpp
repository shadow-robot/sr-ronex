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
#include <realtime_tools/realtime_publisher.h>
#include <sr_common_msgs/GeneralIOState.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x0000000C_GIO_00.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
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

  ros::NodeHandle node_;

  ///The GeneralIO module which is added as a CustomHW to the hardware interface
  boost::shared_ptr<ronex::GeneralIO> general_io_;

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

  int writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length);
  int readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length);

  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  //publisher for the data.
  boost::shared_ptr<realtime_tools::RealtimePublisher<sr_common_msgs::GeneralIOState> > state_publisher_;
  ///Temporary message
  sr_common_msgs::GeneralIOState state_msg_;
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _SR_BOARD_MK2_GIO_HPP_ */

