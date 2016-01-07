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

//! EtherCAT protocol for RoNeX SPI stacker, 02.

#ifndef RONEX_PROTOCOL_0x02000009_DC_MOTOR_SMALL_H_INCLUDED
#define RONEX_PROTOCOL_0x02000009_DC_MOTOR_SMALL_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif

#define RONEX_COMMAND_02000009_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the SPI clock.
#define RONEX_COMMAND_02000009_ADC_SAMPLE_RATE_HZ               1000        //!< Maximum possible ADC sample rate. Don't send EtherCAT packets faster than this.
#define NUM_ANALOGUE_INPUTS                                        6        
#define ANALOGUE_INPUT_RESOLUTION                                 12        //!< 
#define ANALOGUE_INPUT_JUSTIFICATION                           RIGHT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                          RIGHT
#define NUM_DIGITAL_IO                                             6
#define PRODUCT_NAME                                 "dc_motor_small"
#define PRODUCT_ID                                        0x02000009
#define MAXIMUM_NUM_STACKERS                                       2
#define STACKER_TYPE                                               4            //!< range [1..13]


//! Command Types
//! -------------
//! COMMAND_TYPE values are sent by the host to tell the node
//! the type of data contained in the COMMAND struct, or to
//! request specific info from the node.
//! The node will always return the same command_type in its status
//! packet.
//!
#define RONEX_COMMAND_02000009_COMMAND_TYPE_INVALID             0x0000      //!< Zeros imply a failed EtherCAT packet, so this it taken to be invalid.
#define RONEX_COMMAND_02000009_COMMAND_TYPE_NORMAL              0x0001      //!< This is for normal operation.
#define RONEX_COMMAND_02000009_COMMAND_TYPE_ERROR               0x0002      //!< This is for normal operation.



//! Flags
//! -----
//! Available in RONEX_STATUS_02000009.config_info.flags
//! To receive this information, use RONEX_COMMAND_02000009_COMMAND_TYPE_CONFIG_INFO
//! in the RONEX_COMMAND_02000009 struct
//!
#define RONEX_02000009_FLAGS_STACKER_0_PRESENT                  0x1000
#define RONEX_02000009_FLAGS_STACKER_1_PRESENT                  0x2000
#define RONEX_02000009_FLAGS_STACKER_2_PRESENT                  0x4000
#define RONEX_02000009_FLAGS_STACKER_3_PRESENT                  0x8000
#define RONEX_02000009_FLAGS_STACKER_0_ERROR                    0x0100
#define RONEX_02000009_FLAGS_STACKER_1_ERROR                    0x0200
#define RONEX_02000009_FLAGS_STACKER_2_ERROR                    0x0400
#define RONEX_02000009_FLAGS_STACKER_3_ERROR                    0x0800
#define RONEX_02000009_FLAGS_RESERVED_ERRORS                    0x00FC
#define RONEX_02000009_FLAGS_OVER_TEMPERATURE_ERROR             0x0002
#define RONEX_02000009_FLAGS_UNKNOWN_ERROR                      0x0001
                                                                
#define RONEX_02000009_MOTOR_STATUS_FLAG_R_UP                   0x0001
#define RONEX_02000009_MOTOR_STATUS_FLAG_R_DOWN                 0x0002
#define RONEX_02000009_MOTOR_STATUS_FLAG_I_UP                   0x0004
#define RONEX_02000009_MOTOR_STATUS_FLAG_I_DOWN                 0x0008
                                                                
#define RONEX_02000009_MOTOR_COMMAND_FLAG_DIRECTION_FORE        0x0001
#define RONEX_02000009_MOTOR_COMMAND_FLAG_DIRECTION_BACK        0x0000
#define RONEX_02000009_MOTOR_COMMAND_FLAG_WAKE                  0x0002
#define RONEX_02000009_MOTOR_COMMAND_FLAG_SLEEP                 0x0000
#define RONEX_02000009_MOTOR_COMMAND_FLAG_MODE_BRAKE            0x0004
#define RONEX_02000009_MOTOR_COMMAND_FLAG_MODE_NO_BRAKE         0x0000
#define RONEX_02000009_MOTOR_COMMAND_FLAG_INDEX_SIGN_POS        0x0000
#define RONEX_02000009_MOTOR_COMMAND_FLAG_INDEX_SIGN_NEG        0x0008
#define RONEX_02000009_MOTOR_COMMAND_FLAG_INDEX_ENABLE          0x0000
#define RONEX_02000009_MOTOR_COMMAND_FLAG_INDEX_DISABLE         0x0010
#define RONEX_02000009_MOTOR_COMMAND_FLAG_MOTOR_KILL_ENABLE     0x0020
#define RONEX_02000009_MOTOR_COMMAND_FLAG_MOTOR_KILL_DISABLE    0x0000

typedef struct
{
    int16u period;
    int16u onTime;
    int16u flags;
}__attribute__((packed)) MotorPacketCommand;        // 6 bytes

typedef struct
{
    int16u quadrature;
    int16u flags;
}__attribute__((packed)) MotorPacketStatus;         // 4 bytes

// =========================
//     Command Structure
// =========================
typedef struct
{
    int16u              command_type;               // 2
    int16u              pin_output_states_DIO;      // 2
    MotorPacketCommand  motor_packet_command[2];    // 6x2
}__attribute__((packed)) RONEX_COMMAND_02000009;    // 16 bytes



// ======================================================
// This is the packet that's actually sent back to the PC
//   containing either the Status data or Config data
// ======================================================
typedef struct
{
    int16u              command_type;               //  2         // This is a copy of the value in the command structure

    int16u              pin_input_states_DIO;       //  2
    int16u              analogue_in[6];             // 12
    MotorPacketStatus   motor_packet_status[2];     // 4x2
    
}__attribute__((packed)) RONEX_STATUS_02000009;     // 24 bytes


#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RONEX_COMMAND_02000009))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RONEX_COMMAND_02000009)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RONEX_STATUS_02000009))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RONEX_STATUS_02000009)/2)

#define COMMAND_ARRAY_EXPECTED_SIZE_BYTES       16
#define STATUS_ARRAY_EXPECTED_SIZE_BYTES        24
                                                                            // Queued (Mailbox)
                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                           //  Synchronous communication
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_02000009) *4) //!< ET1200 address containing the Status  Structure

#define RONEX_COMMAND_STRUCT        RONEX_COMMAND_02000009                  //!< Required for et1200_interface.h to be generic
#define RONEX_STATUS_STRUCT         RONEX_STATUS_02000009                   //!< Required for et1200_interface.h to be generic



#endif
