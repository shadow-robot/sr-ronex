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

#ifndef RONEX_PROTOCOL_0x02000008_ADC16_H_INCLUDED
#define RONEX_PROTOCOL_0x02000008_ADC16_H_INCLUDED

#include "typedefs_shadow.h"

#if defined(__GNUC__)

#else
    #define __attribute__(x)
#endif

#define RONEX_COMMAND_02000008_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the SPI clock.
#define RONEX_COMMAND_02000008_ADC_SAMPLE_RATE_HZ               1000        //!< Maximum possible ADC sample rate. Don't send EtherCAT packets faster than this.
#define NUM_ANALOGUE_INPUTS                                        6        
#define ANALOGUE_INPUT_RESOLUTION                                 12        //!< 
#define ANALOGUE_INPUT_JUSTIFICATION                           RIGHT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                          RIGHT
#define NUM_DIGITAL_IO                                             6
#define PRODUCT_NAME                                         "adc16"
#define PRODUCT_ID                                        0x02000008
#define MAXIMUM_NUM_STACKERS                                       3
#define STACKER_TYPE                                               3            //!< range [1..13]
#define NUM_ADC16_INPUTS                                          48
#define ADC16_INPUT_RESOLUTION                                    16
#define ADC16_INPUT_SAMPLE_RATE                                 1000


//! Command Types
//! -------------
//! COMMAND_TYPE values are sent by the host to tell the node
//! the type of data contained in the COMMAND struct, or to
//! request specific info from the node.
//! The node will always return the same command_type in its status
//! packet.
//!
#define RONEX_COMMAND_02000008_COMMAND_TYPE_INVALID         0x0000      //!< Zeros imply a failed EtherCAT packet, so this it taken to be invalid.
#define RONEX_COMMAND_02000008_COMMAND_TYPE_NORMAL          0x0001      //!< This is for normal operation.
#define RONEX_COMMAND_02000008_COMMAND_TYPE_WRITE_REG       0x0002      //!< This is used to configure the single ended / differential inputs
#define RONEX_COMMAND_02000008_COMMAND_TYPE_CONFIG_INFO     0x0004      //!< This requests a CONFIG_INFO_02000008 block from the node.
#define RONEX_COMMAND_02000008_COMMAND_TYPE_ERROR           0x00FF      //!< If this is returned from the node, then some kind of error has happened.



//! Flags
//! -----
//! Available in RONEX_STATUS_02000008.config_info.flags
//! To receive this information, use RONEX_COMMAND_02000008_COMMAND_TYPE_CONFIG_INFO
//! in the RONEX_COMMAND_02000008 struct
//!
#define RONEX_02000008_FLAGS_STACKER_0_PRESENT                0x1000
#define RONEX_02000008_FLAGS_STACKER_1_PRESENT                0x2000
#define RONEX_02000008_FLAGS_STACKER_2_PRESENT                0x4000
#define RONEX_02000008_FLAGS_STACKER_3_PRESENT                0x8000
#define RONEX_02000008_FLAGS_STACKER_0_ERROR                  0x0100
#define RONEX_02000008_FLAGS_STACKER_1_ERROR                  0x0200
#define RONEX_02000008_FLAGS_STACKER_2_ERROR                  0x0400
#define RONEX_02000008_FLAGS_STACKER_3_ERROR                  0x0800
#define RONEX_02000008_FLAGS_RESERVED_ERRORS                  0x00FC
#define RONEX_02000008_FLAGS_OVER_TEMPERATURE_ERROR           0x0002
#define RONEX_02000008_FLAGS_UNKNOWN_ERROR                    0x0001



//! Implemented features
//! --------------------
//! Due to the complexity of the requirements for this module, and the need to
//! have something working very quickly, not all of the features will be
//! implemented immediately. Essential features will be implemented first,
//! with less important ones coming later.
//!
//! You can ask the node which features are implemented using the
//! RONEX_COMMAND_02000008_COMMAND_TYPE_CONFIG_INFO. The returned
//! CONFIG_INFO_02000008 contains the implemented_features word.
//!
#define IMPLEMENTED_FEATURE_ANALOGUE_INPUTS                   0x0001
#define IMPLEMENTED_FEATURE_DIGITAL_IO                        0x0002
#define IMPLEMENTED_FEATURE_ADC16_READ_SINGLE_ENDED           0x0004
#define IMPLEMENTED_FEATURE_ADC16_READ_DIFFERENTIAL           0x0008
#define IMPLEMENTED_FEATURE_ADC16_SENSOR_BIAS                 0x0010


// =========================
//     Command Structure
// =========================
typedef struct
{
    int16u      command_type;
    int16u      command_data_0;
    int16u      command_data_1;
    int16u      command_data_2;
    int16u      pin_output_states;
}__attribute__((packed)) RONEX_COMMAND_02000008;


// =========================
//     Status Structure
// =========================
typedef struct
{
    int16u      pin_input_states_DIO;
    int16u      analogue_in[6];
    int16u      adc_mode;
    union16     adc16[48];
}STATUS_DATA_02000008;


// =============================
// Alternative Status Structure
// containing config information
// =============================
typedef struct
{
    int32u    implemented_features;
    int16u    flags;
    int8u     padding[sizeof(STATUS_DATA_02000008) - (sizeof(int32u)+sizeof(int16u))];
}CONFIG_INFO_02000008;


// ======================================================
// This is the packet that's actually sent back to the PC
//   containing either the Status data of Config data
// ======================================================
typedef struct
{
    int16u    command_type;                             // This is a copy of the value in the command structure

    union                                               // The returned status packet contains either
    {                                                   // 
        STATUS_DATA_02000008  status_data;              //      - Actual sensor data
        CONFIG_INFO_02000008  config_info;              //      - Or configuration info
    }info_type;
    
}__attribute__((packed)) RONEX_STATUS_02000008;


#define COMMAND_ARRAY_SIZE_BYTES    (sizeof(RONEX_COMMAND_02000008))
#define COMMAND_ARRAY_SIZE_WORDS    (sizeof(RONEX_COMMAND_02000008)/2)
#define STATUS_ARRAY_SIZE_BYTES     (sizeof(RONEX_STATUS_02000008))
#define STATUS_ARRAY_SIZE_WORDS     (sizeof(RONEX_STATUS_02000008)/2)


                                                                            // Queued (Mailbox)
                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define PROTOCOL_TYPE   EC_QUEUED                                           //  Synchronous communication
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_02000008) *4) //!< ET1200 address containing the Status  Structure

#define RONEX_COMMAND_STRUCT        RONEX_COMMAND_02000008                  //!< Required for et1200_interface.h to be generic
#define RONEX_STATUS_STRUCT         RONEX_STATUS_02000008                   //!< Required for et1200_interface.h to be generic



#endif