/**
 * @file   sr_board_mk2_gio.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 11 11:56:43 2013
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
 */

#include <sr_ronex_ethercat_drivers/sr_board_mk2_gio.hpp>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

#include <math.h>

PLUGINLIB_EXPORT_CLASS(SrBoardMk2GIO, EthercatDevice);

SrBoardMk2GIO::SrBoardMk2GIO() :
    StandardEthercatDevice()
{
}

SrBoardMk2GIO::~SrBoardMk2GIO()
{
  //delete sh_->get_fmmu_config();
  //delete sh_->get_pd_config();
}

void SrBoardMk2GIO::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  StandardEthercatDevice::construct(sh,start_address);

  n_digital_outputs = NUM_DIGITAL_IO;
  n_digital_inputs = NUM_DIGITAL_IO;
  n_analog_outputs = NUM_ANALOGUE_OUTPUTS;
  n_analog_inputs = NUM_ANALOGUE_INPUTS;
  n_PWM_outputs = NUM_PWM_MODULES;

  command_base_ = start_address;
  command_size_ = sizeof(RONEX_COMMAND_0000000C);

  start_address += command_size_;

  status_base_ = start_address;
  status_size_ = sizeof(RONEX_STATUS_0000000C);

  start_address += status_size_;


  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the board
  //
  ROS_INFO("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_, command_size_,
           static_cast<int>(COMMAND_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,                                                  // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                                                           // Logical Start Bit
                                      0x07,                                                           // Logical End Bit
                                      COMMAND_ADDRESS,                                  // Physical Start Address   (in ET1200 address space?)
                                      0x00,                                                           // Physical Start Bit
                                      false,                                                          // Read Enable
                                      true,                                                           // Write Enable
                                      true                                                            // Channel Enable
                                     );


  // WARNING!!!
  // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode EC_QUEUED, the so called mailbox mode)

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the board
  //
  ROS_INFO("Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", status_base_, status_size_,
           static_cast<int>(COMMAND_ADDRESS + command_size_) );
  EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      COMMAND_ADDRESS + (command_size_ * 4),
                                      0x00,
                                      true,
                                      false,
                                      true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  (*pd)[0] = EC_SyncMan(COMMAND_ADDRESS,                             command_size_,    EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(COMMAND_ADDRESS + (command_size_ * 4),              status_size_,     EC_BUFFERED);


  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent    = true;

  (*pd)[1].ChannelEnable = true;

  sh->set_pd_config(pd);

  ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);

  ROS_INFO("Finished constructing the SrBoardMk2GIO driver");
}

int SrBoardMk2GIO::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  StandardEthercatDevice::initialize(hw, allow_unprogrammed);

  return 0;
}

int SrBoardMk2GIO::readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length)
{
  return EthercatDevice::readData(com, address, data, length, FIXED_ADDR);
}


int SrBoardMk2GIO::writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length)
{
  return EthercatDevice::writeData(com, sh_, address, data, length, FIXED_ADDR);
}

void SrBoardMk2GIO::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_0000000C* command = (RONEX_COMMAND_0000000C*)(buffer);
}

bool SrBoardMk2GIO::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_0000000C* status_data = (RONEX_STATUS_0000000C *)(this_buffer+  command_size_);

  ROS_ERROR_STREAM("status data: " << status_data);
  if (status_data->digital_in & RONEX_0000000C_STACKER_0_PRESENT)
  {
    ROS_INFO("A stacker board is plugged in");
  }
  else
  {
    ROS_INFO("No stacker board plugged in");
  }

  return true;
}



/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
