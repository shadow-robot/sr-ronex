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
 * @file   cod_decod_std_io.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  This is an intermediate CodDecod class for any ethercat slave device 
 *         that only has digital and analog (16bit) inputs and outputs.
 *         It decodes and encodes the status and commands, publishes the inputs 
 *         of the device to a topic and reads commands from a topic to set the 
 *         outputs of the device (this may be incompatible with the system functionality 
 *         and will probably have to be removed). This object is not interacting in any 
 *         way with the HardwareInterface. Its children (e.g. cod_decod_01) will.
 **/

#ifndef _COD_DECOD_STD_IO_HPP_
#define _COD_DECOD_STD_IO_HPP_

#include "sr_ronex_drivers/cod_decod/cod_decod.hpp"
#include <sr_ronex_msgs/BoolArray.h>
#include <std_msgs/UInt16MultiArray.h>
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_publisher.h"
#include <vector>
using namespace std;

namespace sr_cod_decod_std_io
{
  enum DigitalIo
  {
    DigitalIo0,
    DigitalIo1,
    DigitalIo2,
    DigitalIo3
  };

  enum AnalogOutput
  {
    AnalogOutput0,
    AnalogOutput1
  };

  enum AnalogInput
  {
    AnlogInput0,
    AnlogInput1,
    AnlogInput2,
    AnlogInput3
  };
}

namespace sr_cod_decod
{
  class CodDecodStdIo: public CodDecod
  {
  public:
    CodDecodStdIo();

    virtual void construct(hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs);
    /*!
     * \brief Reads the information from the ethercat device and decodes it. It's stored in
     * d_in_ and a_in_;
     *
     * \param status_buffer a pointer to the buffer containing the info to read
     */
    virtual void update(unsigned char *status_buffer);
    /*!
     * \brief Builds the command for an ethercat device. It contains the output values currently contained in
     * digital_output_ , analog_output_ and PWM_output_
     *
     * \param command_buffer a pointer to the buffer where the command will be written
     */
    virtual void build_command(unsigned char *command_buffer);
    virtual void add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                     diagnostic_updater::DiagnosticStatusWrapper &d);

    /*!
     * \brief Configures a digital I/O pin as digital input
     *
     * \param pin the physical digital I/O pin of the ethercat module
     */
    virtual void setPinAsDigitalInput(sr_cod_decod_std_io::DigitalIo pin);

    /*!
     * \brief returns the value read from a digital input of the ethercat module
     *
     * \param input_pin the physical digital I/O pin of the ethercat module
     * \return the value read from the digital input
     */
    virtual bool digitalInputToBool(sr_cod_decod_std_io::DigitalIo input_pin);

    /*!
     * \brief Maps the value read from a digital input of the ethercat module to a bool variable
     *
     * \param input_pin the physical digital I/O pin of the ethercat module
     * \param where_to_store_it a pointer to a bool variable where we want to write the read value
     */
    virtual void digitalInputToBool(sr_cod_decod_std_io::DigitalIo input_pin, bool *where_to_store_it);

    /*!
     * \brief returns the value read from an analog input of the ethercat module
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \return the value read from the analog input
     */
    virtual double analogInputToDouble(sr_cod_decod_std_io::AnalogInput input_pin);

    /*!
     * \brief Maps the value read from an analog input of the ethercat module to a double variable
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \param where_to_store_it a pointer to a double variable where we want to write the read value
     */
    virtual void analogInputToDouble(sr_cod_decod_std_io::AnalogInput input_pin, double *where_to_store_it);

    /*!
     * \brief returns the value read from an analog input of the ethercat module
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \return the value read from the analog input
     */
    uint16_t analogInputToUint16(sr_cod_decod_std_io::AnalogInput input_pin);

    /*!
     * \brief Maps the value read from an analog input of the ethercat module to a double variable
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \param where_to_store_it a pointer to a uint16 variable where we want to write the read value
     */
    virtual void analogInputToUint16(sr_cod_decod_std_io::AnalogInput input_pin, uint16_t *where_to_store_it);

    /*!
     * \brief returns the value read from an analog input of the ethercat module
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \return the value read from the analog input
     */
    int analogInputToInt(sr_cod_decod_std_io::AnalogInput input_pin);

    /*!
     * \brief Maps the value read from an analog input of the ethercat module to a double variable
     *
     * \param input_pin the physical analog input pin of the ethercat module
     * \param where_to_store_it a pointer to a int variable where we want to write the read value
     */
    virtual void analogInputToInt(sr_cod_decod_std_io::AnalogInput input_pin, int *where_to_store_it);

    /*!
     * \brief Maps the sign of a double value to a digital output of the ethercat module
     *
     * \param output_pin the physical digital I/O pin of the ethercat module
     * \param value a number whose sign will determine the value of the digital output
     */
    virtual void signToDigitalOutput(sr_cod_decod_std_io::DigitalIo output_pin, double value);
    /*!
     * \brief Maps the sign of a double value to a digital output of the ethercat module
     *
     * \param output_pin the physical digital I/O pin of the ethercat module
     * \param value determines the value of the digital output
     */
    virtual void boolToDigitalOutput(sr_cod_decod_std_io::DigitalIo output_pin, bool value);

    /*!
     * \brief Sets an analog output using a value from a double variable
     *
     * \param output_pin the physical analog output pin of the ethercat module
     * \param value the value to set the output
     */
    virtual void doubleToAnalogOutput(sr_cod_decod_std_io::AnalogOutput output_pin, double value);

    /*!
     * \brief Sets an analog output using a value from a uint16 variable
     *
     * \param output_pin the physical analog output pin of the ethercat module
     * \param value the value to set the output
     */
    virtual void uint16ToAnalogOutput(sr_cod_decod_std_io::AnalogOutput output_pin, uint16_t value);

    /*!
     * \brief Sets a PWM output using PWM_ON_time and PWM_period to determine the PWM signal form
     *
     * \param output_pin the physical digital I/O pin of the ethercat module
     * \param PWM_period the period of the PWM signal from 0x0000 to 0xFFFE
     * \param PWM_ON_time the ON time of the PWM signal from 0x0000 to 0xFFFF (effectively from 0 to PWM_period+1 )
     */
    virtual void doubleToPWMOutput(sr_cod_decod_std_io::DigitalIo output_pin, uint16_t PWM_period, double PWM_ON_time);

    /*!
     * \brief Sets a PWM output using PWM_ON_time and PWM_period to determine the PWM signal form
     *
     * \param output_pin the physical digital I/O pin of the ethercat module
     * \param PWM_period the period of the PWM signal from 0x0000 to 0xFFFE
     * \param PWM_ON_time the ON time of the PWM signal from 0x0000 to 0xFFFF (effectively from 0 to PWM_period+1 )
     */
    virtual void uint16ToPWMOutput(sr_cod_decod_std_io::DigitalIo output_pin, uint16_t PWM_period, uint16_t PWM_ON_time);

  protected:
    //This will be used for the moment. If we have inputs and outputs that don't fit this digital and analog i/o array model
    //then we'll have to change it and probably move this extract and publish stage to the different CodDecod children classes.
    unsigned n_digital_outputs_;
    unsigned n_analog_outputs_;
    unsigned n_digital_inputs_;
    unsigned n_analog_inputs_;
    unsigned n_PWM_outputs_;

    unsigned int command_size_;
    unsigned int status_size_;

    void digitalOutputCommandCB(const sr_ronex_msgs::BoolArrayConstPtr& msg);
    void analogOutputCommandCB(const std_msgs::UInt16MultiArrayConstPtr& msg);
    void PWMOutputCommandCB(const std_msgs::UInt16MultiArrayConstPtr& msg);


    boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::BoolArray> > digital_input_state_publisher_;
    boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::UInt16MultiArray> >analog_input_state_publisher_;

    realtime_tools::RealtimeBox<boost::shared_ptr<sr_ronex_msgs::BoolArray> > digital_output_;
    realtime_tools::RealtimeBox<boost::shared_ptr<std_msgs::UInt16MultiArray> > analog_output_;
    realtime_tools::RealtimeBox<boost::shared_ptr<std_msgs::UInt16MultiArray> > PWM_output_;

    sr_ronex_msgs::BoolArray d_in_;
    std_msgs::UInt16MultiArray a_in_;

    ros::Subscriber sub_digital_output_command_;
    ros::Subscriber sub_analog_output_command_;
    ros::Subscriber sub_PWM_output_command_;

    ros::NodeHandle node_;


  };
}


#endif
