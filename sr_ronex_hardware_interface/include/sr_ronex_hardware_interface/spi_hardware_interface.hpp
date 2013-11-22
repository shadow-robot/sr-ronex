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
 * @file   spi_hardware_interface.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  The RoNeX SPI module data to be stored
 *         in the HardwareInterface.
 **/

#ifndef _SPI_HARDWARE_INTERFACE_H_
#define _SPI_HARDWARE_INTERFACE_H_

#include <pr2_hardware_interface/hardware_interface.h>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000002_SPI_00.h>
#include <vector>

namespace ronex
{
  /**
   * This contains the command and state to be stored as a
   *  CustomHW in the pr2 hardware interface. Makes it possible
   *  to access the RoNeX data from the controllers as well as
   *  the driver.
   *
   * The command and state are a slightly higher level representation
   *  of the definitions used in the protocol header:
   *    sr_ronex_external_protocol/Ronex_Protocol_0x02000002_GIO_00.h
   */
  class SPI
    : public pr2_hardware_interface::CustomHW
  {
  public:
    RONEX_STATUS_02000002 state_;
    RONEX_COMMAND_02000002 command_;
  };
}
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_HARDWARE_INTERFACE_H_ */
