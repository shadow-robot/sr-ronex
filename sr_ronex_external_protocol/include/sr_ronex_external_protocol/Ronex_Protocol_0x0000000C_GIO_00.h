//! EtherCAT protocol for RoNeX General I/O stacker, 01.
//! Works on Node revision 01


#include "typedefs_shadow.h"

#if defined(__GCC__)

#else
    #define __attribute__(x)
#endif


                                                                            // PSoC Hardware Definitions
                                                                            // -------------------------
#define RONEX_COMMAND_0000000C_MASTER_CLOCK_SPEED_HZ        64000000        //!< Master clock. This is divided down to create the PWM clock.
#define RONEX_COMMAND_0000000C_ADC_SAMPLE_RATE_HZ               1000        //!< Maximum possible ADC sample rate. Don't send EtherCAT packets faster than this.
#define NUM_ANALOGUE_INPUTS                                       12        
#define ANALOGUE_INPUT_RESOLUTION                                 12        //!< 
#define ANALOGUE_INPUT_JUSTIFICATION                            LEFT
#define NUM_ANALOGUE_OUTPUTS                                       0
#define ANALOGUE_OUTPUT_RESOLUTION                                 0
#define ANALOGUE_OUTPUT_JUSTIFICATION                           LEFT
#define NUM_DIGITAL_IO                                            12
#define NUM_PWM_MODULES                            (NUM_DIGITAL_IO/2)
#define PRODUCT_NAME                                   "General I/O"
#define PRODUCT_ID                                        0x0000000C
#define MAXIMUM_NUM_STACKERS                                       1

#define RONEX_0000000C_FLAGS_STACKER_0_PRESENT                0x1000
#define RONEX_0000000C_FLAGS_STACKER_1_PRESENT                0x2000
#define RONEX_0000000C_FLAGS_STACKER_2_PRESENT                0x4000
#define RONEX_0000000C_FLAGS_STACKER_3_PRESENT                0x8000
#define RONEX_0000000C_FLAGS_STACKER_0_ERROR                  0x0100
#define RONEX_0000000C_FLAGS_STACKER_1_ERROR                  0x0200
#define RONEX_0000000C_FLAGS_STACKER_2_ERROR                  0x0400
#define RONEX_0000000C_FLAGS_STACKER_3_ERROR                  0x0800

                                                                            //!< The divider for the PWM clock. By adjusting this divider, we have
                                                                            //!  access to a much wider range of PWM frequencies, from 32MHz
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_64_MHZ       1           //!  right down to 1.9Hz. This feature was added so that people could
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_32_MHZ       2           //!  control RC servos, which require 50Hz control frequency.
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_16_MHZ       4           //!  For RC Servos, set Clock Speed = 2MHz, and PWM period to 39999. 
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_08_MHZ       8           //!  This gives 20ms period.
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_04_MHZ      16
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_02_MHZ      32
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_01_MHZ      64

#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_500_KHZ    128
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_250_KHZ    256
#define     RONEX_COMMAND_0000000C_PWM_CLOCK_SPEED_125_KHZ    512




#ifndef EC_BUFFERED 
    #define EC_BUFFERED 1
#endif

#ifndef EC_QUEUED
    #define EC_QUEUED   2
#endif
                                                                                // EtherCAT Protocol
                                                                                // =================
                                                                            
#define PROTOCOL_TYPE   EC_BUFFERED                                             // Asynchronous communication
//#define PROTOCOL_TYPE   EC_QUEUED                                             //  Synchronous communication

#if PROTOCOL_TYPE == EC_BUFFERED
                                                                                // Syncmanager Definitions
                                                                                // -----------------------
    #define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
    #define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_0000000C)*4)  //!< ET1200 address containing the Status  Structure

    #define STATUS_ARRAY_SIZE_BYTES (sizeof(RONEX_STATUS_0000000C))
    #define STATUS_ARRAY_SIZE_WORDS (sizeof(RONEX_STATUS_0000000C)/2)
#endif


#if PROTOCOL_TYPE == EC_QUEUED                                                  // Queued (Mailbox)
                                                                                // Syncmanager Definitions
                                                                                // -----------------------
    #define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
    #define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_0000000C)  )  //!< ET1200 address containing the Status  Structure

    #define STATUS_ARRAY_SIZE_BYTES (sizeof(RONEX_STATUS_0000000C))
    #define STATUS_ARRAY_SIZE_WORDS (sizeof(RONEX_STATUS_0000000C)/2)
#endif






typedef struct                                                              //!< Each PWM module has two outputs. There are six modules, giving 12 outputs total.
{
    int16u  pwm_period;                                                     //!< PWM period is pwm_period/clock_speed.
    int16u  pwm_on_time_0;                                                  //!< On Time is pwm_on_time_0/clock_speed.
    int16u  pwm_on_time_1;
}__attribute__((packed)) RONEX_COMMAND_0000000C_PWM;





typedef struct                                                              //!< Status Structure
{                                                                           //   ----------------
    int16u  analogue_in[12];
    int16u  digital_in;                                                     //!< Bit n: Status of digital pin n.
    int16u  flags;
}__attribute__((packed)) RONEX_STATUS_0000000C;



typedef struct                                                              //! Command structure
{                                                                           //  -----------------
    int16u                                  command_type;
    RONEX_COMMAND_0000000C_PWM              pwm_module[NUM_PWM_MODULES];
    int32u                                  digital_out;                    //!< Bit 0: Direction of digital pin 0, 0=Output, 1=Input
                                                                            //!< Bit 1: Drive     of digital pin 0, 0=Low,    1=High
                                                                            //!< Bit 2: Direction of digital pin 1, 0=Output, 1=Input
                                                                            //!< Bit 3: Drive     of digital pin 1, 0=Low,    1=High
                                                                            //!< etc ..
    int16u                                  pwm_clock_speed;
}__attribute__((packed)) RONEX_COMMAND_0000000C;

