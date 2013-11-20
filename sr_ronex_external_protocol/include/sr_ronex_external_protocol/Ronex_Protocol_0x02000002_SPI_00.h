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


//! Command Types
//! -------------
//! COMMAND_TYPE values are sent by the host to tell the node
//! the type of data contained in the COMMAND struct, or to
//! request specific info from the node.
//! The node will always return the same command_type in its status
//! packet.
//! 
#define RONEX_COMMAND_02000002_COMMAND_TYPE_INVALID           0x0000        //!< Zeros imply a failed EtherCAT packet, so this it taken to be invalid.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_NORMAL            0x0001        //!< This is for normal operation.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_CONFIG_INFO       0x0002        //!< This requests a CONFIG_INFO_02000002 block from the node.
#define RONEX_COMMAND_02000002_COMMAND_TYPE_ERROR             0x00FF        //!< If this is returned from the node, then some kind of error has happened.

#define SPI_MODE_00           0x00
#define SPI_MODE_01           0x01
#define SPI_MODE_10           0x02
#define SPI_MODE_11           0x03


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



//! SPI_Modes
//! ---------
//! Four SPI modes are available.
//! 
#define SPI_MODE_00    0x00
#define SPI_MODE_01    0x01
#define SPI_MODE_10    0x02
#define SPI_MODE_11    0x03



//! Pin output states
//! -----------------
//! 
//! The CS pins and the Digital I/O pins can be controlled by the host.
//! The states can be set before and after the transaction.
//! The CS pins are always outputs

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

typedef struct  
{
    int16u    clock_divider;
    int8u     SPI_mode;
    int8u     inter_byte_gap;
    int8u     data[32];
}__attribute__((packed)) SPI_PACKET_OUT;


typedef struct  
{
    int16u     command_type;

    int16u     pin_output_states_pre;
    int16u     pin_output_states_post;

    SPI_PACKET_OUT spi_out_0;
    SPI_PACKET_OUT spi_out_1;
    SPI_PACKET_OUT spi_out_2;
    SPI_PACKET_OUT spi_out_3;
}__attribute__((packed)) RONEX_COMMAND_02000002;





typedef struct
{
    int8u     pin_input_states_0;
    int8u     pin_input_states_1;
    int8u     pin_input_states_2;
    int8u     pin_input_states_3;

    SPI_PACKET_IN spi_in_0;
    SPI_PACKET_IN spi_in_1;
    SPI_PACKET_IN spi_in_2;
    SPI_PACKET_IN spi_in_3;    
}STATUS_DATA_02000002;

typedef struct
{
    int32u    implemented_features;
    int8u     padding[sizeof(STATUS_02000002_DATA)-4];
}CONFIG_INFO_02000002;

typedef struct  
{
    int8u     data[32];
}__attribute__((packed)) SPI_PACKET_IN;

typedef struct
{
    int16u    command_type;

    union
    {
        STATUS_DATA_02000002  status_data;
        CONFIG_INFO_02000002  config_info;
    }
}__attribute__((packed)) RONEX_STATUS_02000002;



#endif
