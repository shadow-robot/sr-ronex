/**
 * @file   sr_board_0x.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
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
 * @brief Generic driver for a Shadow Robot EtherCAT Slave board.
 * Drivers for every kind of device should inherit from this class.
 *
 *
 */

#include <sr_ronex_drivers/sr_board_0x.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

#include <math.h>

//TODO only for testing purposes (it will be read from eeprom)
#define ETHERCAT_COMMAND_DATA_ADDRESS               0x1000

PLUGINLIB_EXPORT_CLASS(SrBoard0X, EthercatDevice);

SrBoard0X::SrBoard0X() :
    StandardEthercatDevice()
{
}

SrBoard0X::~SrBoard0X()
{
  //delete sh_->get_fmmu_config();
  //delete sh_->get_pd_config();
}

void SrBoard0X::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  StandardEthercatDevice::construct(sh,start_address);

  //TODO only for testing Shadow Robot boards. We read the number of inputs and outputs from the second half of the product code
  //This should ideally be done in standard_ethercat_device in construct, by reading the inputs and outputs
  //parameters from the eeprom of the ethercat device.
  //This is not implemented yet neither in the driver nor in the slave (shadow board) firmware
  n_digital_outputs = ((sh->get_product_code() & 0x00000F00) >>  8) * 2;
  n_digital_inputs = ((sh->get_product_code() & 0x00000F00) >>  8);
  n_analog_outputs = ((sh->get_product_code() & 0x000000F0) >>  4);
  n_analog_inputs = ((sh->get_product_code() & 0x0000000F));
  n_PWM_outputs = n_digital_outputs / 2;

  command_base_ = start_address;
  command_size_ = ((n_digital_outputs/16 + 1) * 2) + (n_PWM_outputs * 4) + (n_analog_outputs * 2);

  start_address += command_size_;

  status_base_ = start_address;
  status_size_ = ((n_digital_inputs/16 + 1) * 2) + (n_analog_inputs * 2);

  start_address += status_size_;


  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the board
  //
  ROS_INFO("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_, command_size_,
           static_cast<int>(ETHERCAT_COMMAND_DATA_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,                                                  // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                                                           // Logical Start Bit
                                      0x07,                                                           // Logical End Bit
                                      ETHERCAT_COMMAND_DATA_ADDRESS,                                  // Physical Start Address   (in ET1200 address space?)
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
           static_cast<int>(ETHERCAT_COMMAND_DATA_ADDRESS + command_size_) );
  EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      ETHERCAT_COMMAND_DATA_ADDRESS + (command_size_ * 4),
                                      0x00,
                                      true,
                                      false,
                                      true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  (*pd)[0] = EC_SyncMan(ETHERCAT_COMMAND_DATA_ADDRESS,                             command_size_,    EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(ETHERCAT_COMMAND_DATA_ADDRESS + (command_size_ * 4),              status_size_,     EC_BUFFERED);


  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent    = true;

  (*pd)[1].ChannelEnable = true;

  sh->set_pd_config(pd);

  ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);

  ROS_INFO("Finished constructing the SrBoard0X driver");
}

int SrBoard0X::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  StandardEthercatDevice::initialize(hw, allow_unprogrammed);

  return 0;
}

int SrBoard0X::readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length)
{
  return EthercatDevice::readData(com, address, data, length, FIXED_ADDR);
}


int SrBoard0X::writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length)
{
  return EthercatDevice::writeData(com, sh_, address, data, length, FIXED_ADDR);
}

void SrBoard0X::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  cod_decod_manager_->build_command(buffer);
}

bool SrBoard0X::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  unsigned char *status_data_ptr;

  //Pointer to the beginning of the status data in the buffer
  status_data_ptr = this_buffer + command_size_;

  cod_decod_manager_->update(status_data_ptr);

  return true;
}


