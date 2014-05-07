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
 * @file   standard_ethercat_device.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  Generic driver for a standard EtherCAT Slave.
 *         Reads information about inputs and outputs from the SII (in eeprom memory) of the slave
 *         Drivers for every kind of standard ethercat device should inherit from this class.
 **/

#include <sr_ronex_drivers/standard_ethercat_device.h>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>

#include <math.h>

PLUGINLIB_EXPORT_CLASS(StandardEthercatDevice, EthercatDevice);

StandardEthercatDevice::StandardEthercatDevice() : EthercatDevice()
{
}

StandardEthercatDevice::~StandardEthercatDevice()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}

void StandardEthercatDevice::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  EthercatDevice::construct(sh,start_address);
  sh->set_fmmu_config( new EtherCAT_FMMU_Config(0) );
  sh->set_pd_config( new EtherCAT_PD_Config(0) );
}

int StandardEthercatDevice::initialize(ros_ethercat_model::RobotState *hw, bool allow_unprogrammed)
{
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();// - hand_->getBridgeRingPosition();

  if((command_size_ > 0) || (status_size_ > 0))
  {
    cod_decod_manager_ = boost::shared_ptr<sr_cod_decod::CodDecodManager>( new sr_cod_decod::CodDecodManager(hw, sh_, n_digital_outputs, n_analog_outputs, n_digital_inputs, n_analog_inputs, n_PWM_outputs) );
  }

  return 0;
}

int StandardEthercatDevice::readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length)
{
  return EthercatDevice::readData(com, address, data, length, FIXED_ADDR);
}


int StandardEthercatDevice::writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length)
{
  return EthercatDevice::writeData(com, sh_, address, data, length, FIXED_ADDR);
}

