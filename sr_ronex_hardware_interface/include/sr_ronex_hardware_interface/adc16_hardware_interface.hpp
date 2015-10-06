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
 * @file   adc16_hardware_interface.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  The RoNeX ADC16 module data to be stored
 *         in the HardwareInterface.
 **/

#ifndef _ADC16_HARDWARE_INTERFACE_H_
#define _ADC16_HARDWARE_INTERFACE_H_

#include <ros_ethercat_model/hardware_interface.hpp>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000008_ADC16_00.h>
#include <vector>

namespace ronex
{
class ADC16Command
{
public:
  std::vector<bool> digital_;
  uint16_t address_;
  std::vector<uint16_t> values_;
  uint16_t command_type_;
};

class ADC16State
{
public:
  std::vector<bool> digital_;
  std::vector<uint16_t> analogue_;
  std::vector<uint16_t> adc_;
  uint16_t address_;
  std::vector<uint16_t> values_;
  uint16_t command_type_;
};

/**
 * This contains the command and state to be stored as a
 *  CustomHW in the pr2 hardware interface. Makes it possible
 *  to access the RoNeX data from the controllers as well as
 *  the driver.
 *
 * The command and state are a slightly higher level representation
 *  of the definitions used in the protocol header:
 *    sr_ronex_external_protocol/Ronex_Protocol_0x02000008_ADC16_00.h
 */
class ADC16
  : public ros_ethercat_model::CustomHW
{
public:
  ADC16State state_;
  ADC16Command command_;
};
}  // namespace ronex
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _ADC16_HARDWARE_INTERFACE_H_ */
