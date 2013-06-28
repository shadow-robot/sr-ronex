/**
 * @file   mk2_gio_hardware_interface.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Jun 27 09:34:37 2013
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
 * @brief The RoNeX mk2 general IO module data to be stored in the HardwareInterface.
 *
 *
 */

#ifndef _MK2_GIO_HARDWARE_INTERFACE_H_
#define _MK2_GIO_HARDWARE_INTERFACE_H_

namespace ronex
{
  class GeneralIOCommand
    : public pr2_hardware_interface::CustomHWCommand
  {
  public:
    std::vector<bool> digital_;

    struct PWM
    {
      unsigned short int period;
      unsigned short int on_time_0;
      unsigned short int on_time_1;
    };

    std::vector<PWM> pwm_;
    unsigned short int pwm_clock_speed_;
  };

  class GeneralIOState
    : public pr2_hardware_interface::CustomHWState
  {
  public:
    std::vector<bool> digital_;
    std::vector<unsigned short int> analogue_;
  };

  /**
   * This contains the command and state to be stored as a
   *  CustomHW in the pr2 hardware interface. Makes it possible
   *  to access the RoNeX data from the controllers as well as
   *  the driver.
   *
   * The command and state are a slightly higher level representation
   *  of the definitions used in the protocol header:
   *    sr_ronex_external_protocol/Ronex_Protocol_0x0000000C_GIO_00.h
   */
  class GeneralIO
    : public pr2_hardware_interface::CustomHW
  {
  public:
    GeneralIOState state_;
    GeneralIOCommand command_;
  };
}
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _MK2_GIO_HARDWARE_INTERFACE_H_ */
