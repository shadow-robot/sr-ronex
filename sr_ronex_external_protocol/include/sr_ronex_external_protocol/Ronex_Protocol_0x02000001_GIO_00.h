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

//! EtherCAT protocol for RoNeX General I/O stacker, 01.
//! Works on Node revision 01

#ifndef RONEX_PROTOCOL_0x02000001_GIO_H_INCLUDED
#define RONEX_PROTOCOL_0x02000001_GIO_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif


                                                                            // PSoC Hardware Definitions
                                                                            // -------------------------
#define RONEX_COMMAND_02000001_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the PWM clock.
#define RONEX_COMMAND_02000001_ADC_SAMPLE_RATE_HZ               1000        //!< Maximum possible ADC sample rate. Don't send EtherCAT packets faster than this.
#define NUM_ANALOGUE_INPUTS                                       12        
#define ANALOGUE_INPUT_RESOLUTION                                 12        //!< 
#define ANALOGUE_INPUT_JUSTIFICATION                           RIGHT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                          RIGHT
#define NUM_DIGITAL_IO                                            12
#define NUM_PWM_MODULES                            (NUM_DIGITAL_IO/2)
#define PRODUCT_NAME                                    "general_IO"
#define PRODUCT_ID                                        0x02000001
#define MAXIMUM_NUM_STACKERS                                       1
#define STACKER_TYPE                                               1            //!< range [1..13]

#define RONEX_COMMAND_02000001_COMMAND_TYPE_INVALID           0x0000        //!< COMMAND_TYPE values are sent by the host to tell the node
#define RONEX_COMMAND_02000001_COMMAND_TYPE_NORMAL            0x0001        //!  the type of data contained in the COMMAND struct.
#define RONEX_COMMAND_02000001_COMMAND_TYPE_ERROR             0x00FF        //!  Currently there is only one type available, NORMAL. The others
                                                                            //!  are considered errors.

#define RONEX_02000001_FLAGS_STACKER_0_PRESENT                0x1000
#define RONEX_02000001_FLAGS_STACKER_1_PRESENT                0x2000
#define RONEX_02000001_FLAGS_STACKER_2_PRESENT                0x4000
#define RONEX_02000001_FLAGS_STACKER_3_PRESENT                0x8000
#define RONEX_02000001_FLAGS_STACKER_0_ERROR                  0x0100
#define RONEX_02000001_FLAGS_STACKER_1_ERROR                  0x0200
#define RONEX_02000001_FLAGS_STACKER_2_ERROR                  0x0400
#define RONEX_02000001_FLAGS_STACKER_3_ERROR                  0x0800
#define RONEX_02000001_FLAGS_RESERVED_ERRORS                  0x00FC
#define RONEX_02000001_FLAGS_OVER_TEMPERATURE_ERROR           0x0002
#define RONEX_02000001_FLAGS_UNKNOWN_ERROR                    0x0001


                                                                            // Queued (Mailbox)
                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                           //  Synchronous communication
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_02000001) *4) //!< ET1200 address containing the Status  Structure

#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RONEX_COMMAND_02000001))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RONEX_COMMAND_02000001)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RONEX_STATUS_02000001 ))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RONEX_STATUS_02000001 )/2)







typedef struct                                                              //!< Each PWM module has two outputs. There are six modules, giving 12 outputs total.
{
    int16u  pwm_period;                                                     //!< PWM period is pwm_period/clock_speed.
    int16u  pwm_on_time_0;                                                  //!< On Time is pwm_on_time_0/clock_speed.
    int16u  pwm_on_time_1;
}__attribute__((packed)) RONEX_COMMAND_02000001_PWM;





typedef struct                                                              //!< Status Structure
{                                                                           //   ----------------
    int16u                          command_type;                           // Copy of command_type from COMMAND struct (NOT USED YET)
    int16u                          analogue_in[12];
    int16u                          digital_in;                             //!< Bit n: Status of digital pin n.
    int16u                          flags;
}__attribute__((packed)) RONEX_STATUS_02000001;



typedef struct                                                              //!< Command structure - Sent by host
{                                                                           //   --------------------------------
    int16u                          command_type;                         //!< Will be a copy of the value sent in the Command structure
    RONEX_COMMAND_02000001_PWM      pwm_module[NUM_PWM_MODULES];
    int32u                          digital_out;                            //!< Bit 0: Direction of digital pin 0, 0=Output, 1=Input
                                                                            //!< Bit 1: Drive     of digital pin 0, 0=Low,    1=High
                                                                            //!< Bit 2: Direction of digital pin 1, 0=Output, 1=Input
                                                                            //!< Bit 3: Drive     of digital pin 1, 0=Low,    1=High
                                                                            //!< etc ..
    int16u                          pwm_clock_divider;
}__attribute__((packed)) RONEX_COMMAND_02000001;

#endif
