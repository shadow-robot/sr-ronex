/**
 * @file   cod_decod.cpp
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
 * @brief This is the parent class for the actual CodDecod objects for every ethercat slave device.
 * The main functions are update, which decodes the data buffer coming from the ethercat device and updates the fields of the HardwareInterface,
 * and build_command, which encodes the command data for the ethercat device (reading from HardwareInterface) and writes it to the buffer.
 *
 */

#include "sr_ronex_ethercat_drivers/cod_decod/cod_decod.hpp"

namespace sr_cod_decod
{
  CodDecod::CodDecod()
  {
  }

  void CodDecod::construct(pr2_hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs)
  {
    sh_ = sh;
    hw_ = hw;
  }


  void CodDecod::update(unsigned char *status_buffer)
  {

  }

  void CodDecod::build_command(unsigned char *command_buffer)
  {

  }

  void CodDecod::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                 diagnostic_updater::DiagnosticStatusWrapper &d)
  {

  }

}
