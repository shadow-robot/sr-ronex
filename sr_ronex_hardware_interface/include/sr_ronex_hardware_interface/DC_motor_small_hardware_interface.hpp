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
 * @file   DC_motor_small_hardware_interface.hpp
 * @author Vahid Aminzadeh <vahid@shadowrobot.com>
 * @brief  The RoNeX small DC motor driver module data to be stored
 *         in the HardwareInterface.
 **/

#ifndef _DC_MOTOR_SMALL_HARDWARE_HARDWARE_INTERFACE_H
#define _DC_MOTOR_SMALL_HARDWARE_HARDWARE_INTERFACE_H

#include <ros_ethercat_model/hardware_interface.hpp>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000009_DC_Motor_Small_00.h>
#include <vector>

namespace ronex
{
class DCMotorCommand
{
public:
  uint16_t command_type_;
  std::vector<bool> digital_;

  struct MotorPacketCommand
  {
    uint16_t period;
    uint16_t on_time;
    uint16_t flags;
  };

  std::vector<MotorPacketCommand> motor_packet_command_;
};

class DCMotorState
{
public:
  uint16_t command_type_;

  std::vector<bool> digital_;
  std::vector<uint16_t> analogue_;

  struct MotorPacketStatus
  {
    int16_t quadrature;
    int16_t flags;
  };
  std::vector<MotorPacketStatus> motor_packet_status_;
};

/**
 * This contains the command and state to be stored as a
 *  CustomHW in the pr2 hardware interface. Makes it possible
 *  to access the RoNeX data from the controllers as well as
 *  the driver.
 *
 * The command and state are a slightly higher level representation
 *  of the definitions used in the protocol header:
 *    sr_ronex_external_protocol/Ronex_Protocol_0x02000001_GIO_00.h
 */
class DCMotor
  : public ros_ethercat_model::CustomHW
{
public:
  DCMotorState state_;
  DCMotorCommand command_;
};
}  // namespace ronex
/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _DC_MOTOR_SMALL_HARDWARE_HARDWARE_INTERFACE_H */
