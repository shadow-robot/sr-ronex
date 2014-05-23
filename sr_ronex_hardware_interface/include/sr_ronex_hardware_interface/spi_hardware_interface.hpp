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

#include <ros_ethercat_model/hardware_interface.hpp>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000002_SPI_00.h>
#include <vector>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

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
    : public ros_ethercat_model::CustomHW
  {
  public:
    SPI()
    {
      state_.reset(new RONEX_STATUS_02000002());
      command_.reset(new RONEX_COMMAND_02000002());
    }

    boost::shared_ptr<RONEX_STATUS_02000002> state_;
    boost::shared_ptr<RONEX_COMMAND_02000002> command_;

    inline void nullify_command(size_t spi_index)
    {
      command_->command_type = RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL;

      //set allocated CS pins to high (pre and post)
      switch( spi_index )
      {
      case 0:
        command_->pin_output_states_pre |= PIN_OUTPUT_STATE_CS_0;
        command_->pin_output_states_post |= PIN_OUTPUT_STATE_CS_0;
        break;

      case 1:
        command_->pin_output_states_pre |= PIN_OUTPUT_STATE_CS_1;
        command_->pin_output_states_post |= PIN_OUTPUT_STATE_CS_1;
        break;

      case 2:
        command_->pin_output_states_pre |= PIN_OUTPUT_STATE_CS_2;
        command_->pin_output_states_post |= PIN_OUTPUT_STATE_CS_2;
        break;

      case 3:
        command_->pin_output_states_pre |= PIN_OUTPUT_STATE_CS_3;
        command_->pin_output_states_post |= PIN_OUTPUT_STATE_CS_3;
        break;
      }

      //setting num bytes to 0 for each SPI outputs
      command_->spi_out[spi_index].num_bytes = 0;
    };
  };
}
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _SPI_HARDWARE_INTERFACE_H_ */
