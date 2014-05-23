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

//! EtherCAT protocol for RoNeX TCAT stacker, 02.

#ifndef RONEX_PROTOCOL_0x02000003_TCAT_H_INCLUDED
#define RONEX_PROTOCOL_0x02000003_TCAT_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif


#define RONEX_COMMAND_02000003_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the SPI clock.
#define NUM_ANALOGUE_INPUTS                                        0
#define ANALOGUE_INPUT_RESOLUTION                                  0        //!<
#define ANALOGUE_INPUT_JUSTIFICATION                           RIGHT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                          RIGHT
#define NUM_DIGITAL_IO                                             0
#define PRODUCT_NAME                                          "tcat"
#define PRODUCT_ID                                        0x02000003
#define MAXIMUM_NUM_STACKERS                                       2
#define STACKER_TYPE                                               2            //!< range [1..13]
#define IMPULSE_RESPONSE_SIZE                                     64
#define NUM_RECEIVERS                                              4
#define NUM_RESERVED_WORDS                                         8
#define PAYLOAD_MAX_SIZE                                          32
#define COMMAND_DATA_MAX_SIZE                                     32

//! Command Types
//! -------------
//! COMMAND_TYPE values are sent by the host to tell the node
//! the type of data contained in the COMMAND struct, or to
//! request specific info from the node.
//! The node will always return the same command_type in its status
//! packet.
//!
#define RONEX_COMMAND_02000003_COMMAND_TYPE_INVALID         0x0000      //!< Zeros imply a failed EtherCAT packet, so this it taken to be invalid.
#define RONEX_COMMAND_02000003_COMMAND_TYPE_NORMAL          0x0001      //!< This is for normal operation.
#define RONEX_COMMAND_02000003_COMMAND_TYPE_CONFIG_INFO     0x0002      //!< This requests a CONFIG_INFO_02000002 block from the node.
#define RONEX_COMMAND_02000003_COMMAND_TYPE_ERROR           0x00FF      //!< If this is returned from the node, then some kind of error has happened.



//! Flags
//! -----
//!
#define RONEX_02000003_FLAGS_STACKER_0_PRESENT                0x1000
#define RONEX_02000003_FLAGS_STACKER_1_PRESENT                0x2000
#define RONEX_02000003_FLAGS_STACKER_2_PRESENT                0x4000
#define RONEX_02000003_FLAGS_STACKER_3_PRESENT                0x8000
#define RONEX_02000003_FLAGS_STACKER_0_ERROR                  0x0100
#define RONEX_02000003_FLAGS_STACKER_1_ERROR                  0x0200
#define RONEX_02000003_FLAGS_STACKER_2_ERROR                  0x0400
#define RONEX_02000003_FLAGS_STACKER_3_ERROR                  0x0800
#define RONEX_02000003_FLAGS_RESERVED_ERRORS                  0x00FC
#define RONEX_02000003_FLAGS_OVER_TEMPERATURE_ERROR           0x0002
#define RONEX_02000003_FLAGS_UNKNOWN_ERROR                    0x0001



//! TCAT Channel Configuration
//! --------------------------
//!
#define TCAT_CHANNEL_1                                  	0x0001
#define TCAT_CHANNEL_2                                  	0x0002
#define TCAT_CHANNEL_3                                  	0x0003
#define TCAT_CHANNEL_4                                  	0x0004
#define TCAT_CHANNEL_5                                  	0x0005
#define TCAT_CHANNEL_7                                  	0x0006

#define TCAT_PREAMBLE_1                                 	0x0010
#define TCAT_PREAMBLE_2                                 	0x0020
#define TCAT_PREAMBLE_3                                 	0x0030
#define TCAT_PREAMBLE_4                                 	0x0040
#define TCAT_PREAMBLE_5                                 	0x0050
#define TCAT_PREAMBLE_6                                 	0x0060
#define TCAT_PREAMBLE_7                                 	0x0070
#define TCAT_PREAMBLE_8                                 	0x0080
#define TCAT_PREAMBLE_LEN_64                               	0x0000
#define TCAT_PREAMBLE_LEN_128                              	0x0100
#define TCAT_PREAMBLE_LEN_256                              	0x0200
#define TCAT_PREAMBLE_LEN_512                              	0x0300
#define TCAT_PREAMBLE_LEN_1024                             	0x0400
#define TCAT_PREAMBLE_LEN_1536                             	0x0500
#define TCAT_PREAMBLE_LEN_2048                             	0x0600
#define TCAT_PREAMBLE_LEN_4096                             	0x0700
#define TCAT_PRF_16MHZ                               		0x0000
#define TCAT_PRF_64MHZ                               		0x0800

#define TCAT_DATA_RATE_110KBPS                         		0x1000
#define TCAT_DATA_RATE_850KBPS                         		0x2000
#define TCAT_DATA_RATE_6810KBPS                        		0x3000

#define TCAT_STD_FRAME                         				0x1000
#define TCAT_NON_STD_FRAME                        			0x2000



//! Implemented features
//! --------------------
//! Due to the complexity of the requirements for this module, and the need to
//! have something working very quickly, not all of the features will be
//! implemented immediately. Essential features will be implemented first,
//! with less important ones coming later.
//!
//! You can ask the node which features are implemented using the
//! RONEX_COMMAND_02000003_COMMAND_TYPE_CONFIG_INFO. The returned
//! CONFIG_INFO_02000003 contains the implemented_features word.
//!
#define IMPLEMENTED_FEATURE_CHANNEL_SETUP                     0x0001




//! Use this to convert the integer FPI value from the Status
//! struct into a floating point value.
//!
#define FPI_FIXED_POINT_TO_FLOAT(fpi)   ((float)fpi * (1.0f/64.0f))


typedef struct
{
    int16s real;
    int16s imaginary;
}IMPULSE_SAMPLE;


//! The RECEIVER_DATA structure contains data from one receiver.
//!
//! impulse_response[] is a subset of the 4064 sample array inside the DW1000 chip.
//! We only send back 64 samples, starting at first_sample_number.
//!
//! FPI is a 10.6 fixed point value. It is a non-integer index into the impulse
//! response date which tells us where the first pulse appears.
//!
typedef struct
{
    int16u          reserved[NUM_RESERVED_WORDS];
    IMPULSE_SAMPLE  impulse_response[IMPULSE_RESPONSE_SIZE];
    int16u          first_sample_number;
    int16u          payload[PAYLOAD_MAX_SIZE];
    int32u          rx_frame_information;
    int16u          std_noise;
    int32u          flags;
    int16u          FPI;
    int32u          timestamp_L;
    int16u          timestamp_H;
}__attribute__((packed)) RECEIVER_DATA;



typedef struct
{
    int16u     command_type;

    int16u     command_number;
    int16u     command_data_size;
    int16u     command_data[COMMAND_DATA_MAX_SIZE];

}__attribute__((packed)) RONEX_COMMAND_02000003;



//! Due to the large size of the data packet, only one receiver's data
//! is sent in each EtherCAT packet.
//! When a message arrives at the base station, data is collected from
//! each in turn, and sent to the host in individual EtherCAT packets.
//! Therefore it takes 4 EtherCAT packets to transmit all of the receiver
//! data. The 4 packets will all have the same sequence_number, but a
//! different receiver_number.
//!
//! command_type:       currently undecided function
//! sequence_number:    When this changes, it signals the arrival of a new message
//! receiver_number:    Which of the 4 receivers is this data from?
//! receiver_data:      Data from one receiver
//!
typedef struct
{
    int16u             command_type;
    int16u             sequence_number;
    int16u             receiver_number;
    RECEIVER_DATA      receiver_data;
}__attribute__((packed)) RONEX_STATUS_02000003;


#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RONEX_COMMAND_02000003))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RONEX_COMMAND_02000003)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RONEX_STATUS_02000003))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RONEX_STATUS_02000003)/2)


                                                                            // Queued (Mailbox)
                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                           //  Synchronous communication
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_02000003) *4) //!< ET1200 address containing the Status  Structure

#define RONEX_COMMAND_STRUCT        RONEX_COMMAND_02000003                  //!< Required for et1200_interface.h to be generic
#define RONEX_STATUS_STRUCT         RONEX_STATUS_02000003                   //!< Required for et1200_interface.h to be generic

#endif
