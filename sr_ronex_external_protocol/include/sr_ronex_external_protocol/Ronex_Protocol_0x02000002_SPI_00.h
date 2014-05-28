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

#ifndef RONEX_PROTOCOL_0x02000002_SPI_H_INCLUDED
#define RONEX_PROTOCOL_0x02000002_SPI_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif

#define RONEX_COMMAND_02000002_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the SPI clock.
#define RONEX_COMMAND_02000002_ADC_SAMPLE_RATE_HZ               1000        //!< Maximum possible ADC sample rate. Don't send EtherCAT packets faster than this.
#define NUM_ANALOGUE_INPUTS                                        6
#define ANALOGUE_INPUT_RESOLUTION                                 12        //!<
#define ANALOGUE_INPUT_JUSTIFICATION                           RIGHT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                          RIGHT
#define NUM_DIGITAL_IO                                             6
#define NUM_DIO_SAMPLES                                            4
#define NUM_SPI_OUTPUTS                                            4
#define PRODUCT_NAME                                           "spi"
#define PRODUCT_ID                                        0x02000002
#define MAXIMUM_NUM_STACKERS                                       2
#define STACKER_TYPE                                               2            //!< range [1..13]
#define SPI_TRANSACTION_MAX_SIZE                                  32


//! Command Types
//! -------------
//! COMMAND_TYPE values are sent by the host to tell the node
//! the type of data contained in the COMMAND struct, or to
//! request specific info from the node.
//! The node will always return the same command_type in its status
//! packet.
//!
#define RONEX_COMMAND_02000002_COMMAND_TYPE_INVALID         0x0000      //!< Zeros imply a failed EtherCAT packet, so this it taken to be invalid.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL          0x0001      //!< This is for normal operation.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_CONFIG_INFO     0x0002      //!< This requests a CONFIG_INFO_02000002 block from the node.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_ERROR           0x00FF      //!< If this is returned from the node, then some kind of error has happened.



//! Flags
//! -----
//! Available in RONEX_STATUS_02000002.config_info.flags
//! To receive this information, use RONEX_COMMAND_02000002_COMMAND_TYPE_CONFIG_INFO
//! in the RONEX_COMMAND_02000002 struct
//!
#define RONEX_02000002_FLAGS_STACKER_0_PRESENT                0x1000
#define RONEX_02000002_FLAGS_STACKER_1_PRESENT                0x2000
#define RONEX_02000002_FLAGS_STACKER_2_PRESENT                0x4000
#define RONEX_02000002_FLAGS_STACKER_3_PRESENT                0x8000
#define RONEX_02000002_FLAGS_STACKER_0_ERROR                  0x0100
#define RONEX_02000002_FLAGS_STACKER_1_ERROR                  0x0200
#define RONEX_02000002_FLAGS_STACKER_2_ERROR                  0x0400
#define RONEX_02000002_FLAGS_STACKER_3_ERROR                  0x0800
#define RONEX_02000002_FLAGS_RESERVED_ERRORS                  0x00FC
#define RONEX_02000002_FLAGS_OVER_TEMPERATURE_ERROR           0x0002
#define RONEX_02000002_FLAGS_UNKNOWN_ERROR                    0x0001



//! SPI Configuration
//! -----------------
//!
//!
//!
#define SPI_CONFIG_MODE_00                                  0x0000
#define SPI_CONFIG_MODE_01                                  0x0001
#define SPI_CONFIG_MODE_10                                  0x0002
#define SPI_CONFIG_MODE_11                                  0x0003
#define SPI_CONFIG_INPUT_TRIGGER_NONE                       0x0000
#define SPI_CONFIG_INPUT_TRIGGER_D0                         0x0004
#define SPI_CONFIG_INPUT_TRIGGER_D1                         0x0008
#define SPI_CONFIG_INPUT_TRIGGER_D2                         0x000c
#define SPI_CONFIG_INPUT_TRIGGER_D3                         0x0010
#define SPI_CONFIG_INPUT_TRIGGER_D4                         0x0014
#define SPI_CONFIG_INPUT_TRIGGER_D5                         0x0018
#define SPI_CONFIG_MOSI_SOMI_DIFFERENT_PIN                  0x0000
#define SPI_CONFIG_MOSI_SOMI_SAME_PIN                       0x0020



//! Implemented features
//! --------------------
//! Due to the complexity of the requirements for this module, and the need to
//! have something working very quickly, not all of the features will be
//! implemented immediately. Essential features will be implemented first,
//! with less important ones coming later.
//!
//! You can ask the node which features are implemented using the
//! RONEX_COMMAND_02000002_COMMAND_TYPE_CONFIG_INFO. The returned
//! CONFIG_INFO_02000002 contains the implemented_features word.
//!
#define IMPLEMENTED_FEATURE_TRANSACTION_SIZE                  0x0001
#define IMPLEMENTED_FEATURE_CLOCK_DIVIDER                     0x0002
#define IMPLEMENTED_FEATURE_PIN_OUTPUTS                       0x0004

#define IMPLEMENTED_FEATURE_SPI_MODE_00                       0x0008
#define IMPLEMENTED_FEATURE_SPI_MODE_01                       0x0010
#define IMPLEMENTED_FEATURE_SPI_MODE_10                       0x0020
#define IMPLEMENTED_FEATURE_SPI_MODE_11                       0x0040
#define IMPLEMENTED_FEATURE_INTER_BYTE_DELAY                  0x0080

#define IMPLEMENTED_FEATURE_INPUT_TRIGGERING                  0x0100
#define IMPLEMENTED_FEATURE_MOSI_SOMI_SAME_PIN                0x0200

#define IMPLEMENTED_FEATURE_ANALOGUE_INPUTS                   0x0400



//! Pin output states
//! -----------------
//!
//! The CS pins and the Digital I/O pins can be controlled by the host.
//! The states can be set before and after the transaction.
//! The CS pins are always outputs
//!
#define PIN_OUTPUT_STATE_DIO_0                0x0001
#define PIN_OUTPUT_DIRECTION_DIO_0            0x0002
#define PIN_OUTPUT_STATE_DIO_1                0x0004
#define PIN_OUTPUT_DIRECTION_DIO_1            0x0008
#define PIN_OUTPUT_STATE_DIO_2                0x0010
#define PIN_OUTPUT_DIRECTION_DIO_2            0x0020
#define PIN_OUTPUT_STATE_DIO_3                0x0040
#define PIN_OUTPUT_DIRECTION_DIO_3            0x0080
#define PIN_OUTPUT_STATE_DIO_4                0x0100
#define PIN_OUTPUT_DIRECTION_DIO_4            0x0200
#define PIN_OUTPUT_STATE_DIO_5                0x0400
#define PIN_OUTPUT_DIRECTION_DIO_5            0x0800
#define PIN_OUTPUT_STATE_CS_0                 0x1000
#define PIN_OUTPUT_STATE_CS_1                 0x2000
#define PIN_OUTPUT_STATE_CS_2                 0x4000
#define PIN_OUTPUT_STATE_CS_3                 0x8000



//! Pin input states
//! ----------------
//!
//! The Digital I/O and MOSI pins are sampled by the PSoC at four
//! points during the SPI transaction. All Digital I/O are sampled
//! even if they are set as outputs.
//!
#define PIN_INPUT_STATE_DIO_0                 0x0001
#define PIN_INPUT_STATE_DIO_1                 0x0002
#define PIN_INPUT_STATE_DIO_2                 0x0004
#define PIN_INPUT_STATE_DIO_3                 0x0008
#define PIN_INPUT_STATE_DIO_4                 0x0010
#define PIN_INPUT_STATE_DIO_5                 0x0020
#define PIN_INPUT_STATE_MOSI_0                0x0001
#define PIN_INPUT_STATE_MOSI_1                0x0002
#define PIN_INPUT_STATE_MOSI_2                0x0004
#define PIN_INPUT_STATE_MOSI_3                0x0008


typedef struct
{
    int16u    clock_divider;
    int16u    SPI_config;
    int8u     inter_byte_gap;
    int8u     num_bytes;
    int8u     data_bytes[SPI_TRANSACTION_MAX_SIZE];
}__attribute__((packed)) SPI_PACKET_OUT;


typedef struct
{
    int16u     command_type;

    int16u     pin_output_states_pre;
    int16u     pin_output_states_post;

    SPI_PACKET_OUT spi_out[NUM_SPI_OUTPUTS];

}__attribute__((packed)) RONEX_COMMAND_02000002;





typedef struct
{
    int8u     data_bytes[SPI_TRANSACTION_MAX_SIZE];
}__attribute__((packed)) SPI_PACKET_IN;

typedef struct
{
    int8u     pin_input_states_DIO[NUM_DIO_SAMPLES];
    int8u     pin_input_states_SOMI[NUM_DIO_SAMPLES];

    SPI_PACKET_IN spi_in[NUM_SPI_OUTPUTS];

    int16u    analogue_in[6];
}STATUS_DATA_02000002;

typedef struct
{
    int32u    implemented_features;
    int16u    flags;
    int8u     padding[sizeof(STATUS_DATA_02000002)-6];
}CONFIG_INFO_02000002;

typedef struct
{
    int16u    command_type;

    union
    {
        STATUS_DATA_02000002  status_data;
        CONFIG_INFO_02000002  config_info;
    }info_type;

}__attribute__((packed)) RONEX_STATUS_02000002;


#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RONEX_COMMAND_02000002))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RONEX_COMMAND_02000002)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RONEX_STATUS_02000002))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RONEX_STATUS_02000002)/2)


                                                                            // Queued (Mailbox)
                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                           //  Synchronous communication
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_02000002) *4) //!< ET1200 address containing the Status  Structure

#define RONEX_COMMAND_STRUCT        RONEX_COMMAND_02000002                  //!< Required for et1200_interface.h to be generic
#define RONEX_STATUS_STRUCT         RONEX_STATUS_02000002                   //!< Required for et1200_interface.h to be generic

#endif
