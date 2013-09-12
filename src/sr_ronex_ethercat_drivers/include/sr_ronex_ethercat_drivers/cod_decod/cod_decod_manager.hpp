/**
 * @file   cod_decod_manager.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 *
 * Copyright 2011 Shadow Robot Company Ltd.
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
 * @brief An instance of this class will be created by the driver of every ethercat slave.
 * Depending on the configuration read from the parameter server (defined in cod_decod_config.yaml) It will instantiate an object of the
 * corresponding subclass of CodDecod.
 * The update and build_command functions will call the corresponding functions of the CodDecod object.
 *
 */

#ifndef _COD_DECOD_MANAGER_HPP_
#define _COD_DECOD_MANAGER_HPP_

#include <boost/smart_ptr.hpp>
#include <pr2_hardware_interface/hardware_interface.h>
#include "sr_ronex_ethercat_drivers/cod_decod/cod_decod.hpp"
#include <al/ethercat_slave_handler.h>
#include <pluginlib/class_loader.h>

namespace sr_cod_decod
{
  class CodDecodManager
  {
  public:
    CodDecodManager(pr2_hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs);
    ~CodDecodManager();

    void update(unsigned char *status_buffer);
    void build_command(unsigned char *command_buffer);

  private:
    ///This points to an object of a CodDecod child class. The actual type is decided in the construct function
    ///based on the declared plugins, and on the product code and serial number read from the ethercat slave
    boost::shared_ptr<CodDecod> cod_decod_;

    ros::NodeHandle node_;
    ///this ClassLoader allows us to get the list of the declared CodDecod classes and load the one that matches
    ///the product code and serial number read from the ethercat slave
    pluginlib::ClassLoader<CodDecod> cod_decod_loader_;
  };
}

#endif
