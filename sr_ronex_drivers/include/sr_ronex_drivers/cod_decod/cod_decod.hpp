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
 * @file   cod_decod.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  This is the parent class for the actual CodDecod objects for every ethercat slave device.
 *         The main functions are update, which decodes the data buffer coming from the ethercat device 
 *         and updates the fields of the HardwareInterface, and build_command, which encodes the command 
 *         data for the ethercat device (reading from HardwareInterface) and writes it to the buffer.
 **/

#ifndef _COD_DECOD_HPP_
#define _COD_DECOD_HPP_

#include <ros_ethercat_model/robot_state.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <ros_ethercat_hardware/ethercat_hardware.h>
#include <pluginlib/class_list_macros.h>

namespace sr_cod_decod
{
  class CodDecod
  {
  public:
    CodDecod();
    virtual ~CodDecod() {}

    virtual void construct(hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs);
    virtual void update(unsigned char *status_buffer);
    virtual void build_command(unsigned char *command_buffer);
    virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                     diagnostic_updater::DiagnosticStatusWrapper &d);
  protected:
    EtherCAT_SlaveHandler *sh_;
    ros_ethercat_model::RobotState *hw_;
  };
}


#endif
