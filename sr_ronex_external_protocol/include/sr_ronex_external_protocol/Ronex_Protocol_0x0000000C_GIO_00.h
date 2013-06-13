//! EtherCAT protocol for RoNeX General I/O stacker, 01.
//! Works on Node revision 01


#include "typedefs_shadow.h"
                                                                            // Hardware Definitions
                                                                            // --------------------
#define RONEX_COMMAND_0000000C_MASTER_CLOCK_SPEED_HZ        64000000
#define RONEX_COMMAND_0000000C_ADC_SAMPLE_RATE_HZ               1000
#define NUM_ANALOGUE_INPUTS                                       12
#define NUM_ANALOGUE_OUTPUTS                                       0
#define NUM_DIGITAL_IO                                            12
#define NUM_PWM_MODULES                            (NUM_DIGITAL_IO/2)
#define RONEX_0000000C_STACKER_0_PRESENT                      0x8000

                                                                            // Syncmanager Definitions
                                                                            // -----------------------
#define COMMAND_ADDRESS 0x1000                                              //!< ET1200 address containing the Command Structure
#define STATUS_ADDRESS  (COMMAND_ADDRESS+sizeof(RONEX_COMMAND_0000000C))    //!< ET1200 address containing the Status  Structure

#define STATUS_ARRAY_SIZE_WORDS (sizeof(RONEX_STATUS_0000000C)/2)

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





typedef struct                                                      //!< Each PWM module has two outputs. There are six modules, giving 12 outputs total.
{
    int16u  pwm_period;                                             //!< PWM period is pwm_period/clock_speed.
    int16u  pwm_on_time_0;                                          //!< On Time is pwm_on_time_0/clock_speed.
    int16u  pwm_on_time_1;
}RONEX_COMMAND_0000000C_PWM;





typedef struct                                                              //!< Status Structure
{                                                                           //   ----------------
    int16u  analogue_in[12];
    int16u  digital_in;                                                     //!< Bit n: Status of digital pin n.
}RONEX_STATUS_0000000C;



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
}RONEX_COMMAND_0000000C;

