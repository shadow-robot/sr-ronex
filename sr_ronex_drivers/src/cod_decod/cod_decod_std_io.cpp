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
 * @file   cod_decod_01.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  This is and example implementation of an actual CodDecod class for a determined 
 *         ethercat slave device. The ethercat device will be determined by its product code 
 *         and by its serial number. The main functions are update, which decodes the data 
 *         buffer coming from the ethercat device and updates the fields of the HardwareInterface,
 *         and build_command, which encodes the command data for the ethercat device (reading 
 *         from HardwareInterface) and writes it to the buffer.
 **/

#include "sr_ronex_drivers/cod_decod/cod_decod_std_io.hpp"
#include <sstream>

PLUGINLIB_EXPORT_CLASS(sr_cod_decod::CodDecodStdIo, sr_cod_decod::CodDecod);

namespace sr_cod_decod
{
  CodDecodStdIo::CodDecodStdIo()
    :CodDecod(),
     n_digital_outputs_(0),
     n_analog_outputs_(0),
     n_digital_inputs_(0),
     n_analog_inputs_(0),
     n_PWM_outputs_(0),
     command_size_(0),
     status_size_(0),
     digital_input_state_publisher_(NULL),
     analog_input_state_publisher_(NULL)
  {
  }

  void CodDecodStdIo::construct(hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs)
  {
    CodDecod::construct(hw, sh, n_digital_outputs, n_analog_outputs, n_digital_inputs, n_analog_inputs, n_PWM_outputs);
    n_digital_outputs_ = n_digital_outputs;
    n_analog_outputs_ = n_analog_outputs;
    n_digital_inputs_ = n_digital_inputs;
    n_analog_inputs_ = n_analog_inputs;
    n_PWM_outputs_ = n_PWM_outputs;

    int dig_out_size = 0;
    if (n_digital_outputs_ > 0 )
      dig_out_size = ((n_digital_outputs_/16 + 1) * 2);

    command_size_ = dig_out_size + (n_PWM_outputs_ * 4) + (n_analog_outputs_ * 2);

    int dig_in_size = 0;
    if (n_digital_inputs_ > 0 )
      dig_in_size = ((n_digital_inputs_/16 + 1) * 2);

    status_size_ = dig_in_size + (n_analog_inputs_ * 2);

    node_ = ros::NodeHandle();

    //Initialise digital outputs to 0
    boost::shared_ptr<sr_ronex_msgs::BoolArray> d_out_ptr(new sr_ronex_msgs::BoolArray());
    d_out_ptr->data.clear();
    for (unsigned i = 0; i < n_digital_outputs_; ++i)
    {
      d_out_ptr->data.push_back(false);
    }
    digital_output_.set(d_out_ptr);

    //Initialise analog outputs to 0
    boost::shared_ptr<std_msgs::UInt16MultiArray> a_out_ptr(new std_msgs::UInt16MultiArray());
    a_out_ptr->data.clear();
    for (unsigned i = 0; i < n_analog_outputs_; ++i)
    {
      a_out_ptr->data.push_back(0x0000);
    }
    analog_output_.set(a_out_ptr);

    //Initialise PWM outputs to 0
    boost::shared_ptr<std_msgs::UInt16MultiArray> PWM_out_ptr(new std_msgs::UInt16MultiArray());
    PWM_out_ptr->data.clear();
    for (unsigned i = 0; i < (n_PWM_outputs_ * 2); ++i)
    {
      PWM_out_ptr->data.push_back(0x0000);
    }
    PWM_output_.set(PWM_out_ptr);


    char buff[200];
    string topic;

    sprintf(buff, "device_0x%08X_0x%08X_digital_outputs_command",
            sh_->get_product_code(),
            sh_->get_serial());
    topic = buff;
    sub_digital_output_command_ = node_.subscribe<sr_ronex_msgs::BoolArray>(topic, 1, &CodDecodStdIo::digitalOutputCommandCB, this);


    sprintf(buff, "device_0x%08X_0x%08X_analog_outputs_command",
            sh_->get_product_code(),
            sh_->get_serial());
    topic = buff;
    sub_analog_output_command_ = node_.subscribe<std_msgs::UInt16MultiArray>(topic, 1, &CodDecodStdIo::analogOutputCommandCB, this);

    sprintf(buff, "device_0x%08X_0x%08X_PWM_outputs_command",
            sh_->get_product_code(),
            sh_->get_serial());
        topic = buff;
    sub_PWM_output_command_ = node_.subscribe<std_msgs::UInt16MultiArray>(topic, 1, &CodDecodStdIo::PWMOutputCommandCB, this);


    sprintf(buff, "device_0x%08X_0x%08X_digital_inputs_state",
            sh_->get_product_code(),
            sh_->get_serial());
    topic = buff;
    digital_input_state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_ronex_msgs::BoolArray>(node_, topic, 1));


    sprintf(buff, "device_0x%08X_0x%08X_analog_inputs_state",
            sh_->get_product_code(),
            sh_->get_serial());
    topic = buff;
    analog_input_state_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::UInt16MultiArray>(node_, topic, 1));


  }

  void CodDecodStdIo::update(unsigned char *status_buffer)
  {
    unsigned char *buff_ptr;

    //The first byte(s) in the buffer contain the digital inputs values
    buff_ptr = status_buffer;

    d_in_.data.clear();

    //Read the digital inputs from the incoming buffer
    for (unsigned i = 0; i < n_digital_inputs_; i++)
    {
      if(buff_ptr[0] & (0x01 << (i % 8)))         d_in_.data.push_back(true);
      else                                        d_in_.data.push_back(false);

      if((i + 1) % 8 == 0) buff_ptr++;
    }

    //Publish to the digital inputs state topic using the Realtime publisher
    if (digital_input_state_publisher_ && digital_input_state_publisher_->trylock())
    {
      digital_input_state_publisher_->msg_= d_in_;
      digital_input_state_publisher_->unlockAndPublish();
    }


    //point to the beginning of the analog inputs in the buffer
    buff_ptr = status_buffer + ((n_digital_inputs_/16 + 1) * 2);

    a_in_.data.clear();

    //Read the analog inputs from the incoming buffer
    for (unsigned i = 0; i < n_analog_inputs_; i++)
    {
      a_in_.data.push_back(*((uint16_t *)buff_ptr));

      buff_ptr += 2;
    }

    //Publish to the analog inputs state topic using the Realtime publisher
    if (analog_input_state_publisher_ && analog_input_state_publisher_->trylock())
    {
      analog_input_state_publisher_->msg_= a_in_;
      analog_input_state_publisher_->unlockAndPublish();
    }

    //for debugging only
    char buff[300];
    char aux[3];
    strcpy(buff, "");
    for (unsigned int i = 0; i<status_size_; i++)
    {
      sprintf(aux, "%02x", static_cast<uint16_t>(status_buffer[i]));
      strcat(buff, aux);
    }
    if(status_size_ > 0)
    {
      ROS_DEBUG("Stat buffer %02d: %s", sh_->get_ring_position(), buff);
    }
  }

  void CodDecodStdIo::build_command(unsigned char *command_buffer)
  {
    unsigned char *buff_ptr;
    buff_ptr = command_buffer;

    //Read the digital outputs from the digital_output_ realtime box and write them on the output buffer
    boost::shared_ptr<sr_ronex_msgs::BoolArray> d_out_ptr;
    digital_output_.get(d_out_ptr);

    //first we set all the digital output bytes in the buffer to zero
    for (size_t i = 0; i < ((n_digital_outputs_/16 + 1) * 2); i++)
    {
      buff_ptr[i] = 0;
    }

    //then we write the actual values (only the digital ones now) to the buffer
    for (size_t i = 0; i < d_out_ptr->data.size(); i++)
    {
      if (d_out_ptr->data.at(i))
        (*((uint8_t *)buff_ptr)) |= (0x01 << (i % 8));
      if((i + 1) % 8 == 0)
        ++buff_ptr;
    }


    //point to the beginning of the PWM outputs in the buffer
    buff_ptr = command_buffer + ((n_digital_outputs_/16 + 1) * 2);

    //Read the PWM outputs from the PWM_output_ realtime box and write them on the output buffer
    boost::shared_ptr<std_msgs::UInt16MultiArray> PWM_out_ptr;
    PWM_output_.get(PWM_out_ptr);

    //we write the actual values to the buffer
    for (size_t i = 0; i < PWM_out_ptr->data.size(); i++)
    {
      *((uint16_t *)buff_ptr) = PWM_out_ptr->data.at(i);
      buff_ptr += 2;
    }


    //point to the beginning of the analog outputs in the buffer
    buff_ptr = command_buffer + ((n_digital_outputs_/16 + 1) * 2) + (n_PWM_outputs_ * 4);

    //Read the analog outputs from the analog_output_ realtime box and write them on the output buffer
    boost::shared_ptr<std_msgs::UInt16MultiArray> a_out_ptr;
    analog_output_.get(a_out_ptr);

    //we write the actual values to the buffer
    for (size_t i = 0; i < a_out_ptr->data.size(); i++)
    {
      *((uint16_t *)buff_ptr) = a_out_ptr->data.at(i);
      buff_ptr += 2;
    }

    //for debugging only
    char buff[300];
    char aux[3];
    strcpy(buff, "");
    for (unsigned int i = 0; i<command_size_; i++)
    {
      sprintf(aux, "%02x", static_cast<uint16_t>(command_buffer[i]));
      strcat(buff, aux);
    }
    if(command_size_ > 0)
    {
      ROS_DEBUG("Cmd buffer %02d: %s", sh_->get_ring_position(), buff);
    }
    //ROS_INFO("Buffer: 0x%02x", static_cast<uint16_t>(buffer[0]));
    //ROS_INFO("State: %02d", sh_->get_state());

  }

  void CodDecodStdIo::digitalOutputCommandCB(const sr_ronex_msgs::BoolArrayConstPtr& msg)
  {
    if(msg->data.size() == n_digital_outputs_)
    {
      boost::shared_ptr<sr_ronex_msgs::BoolArray> d_out_ptr(new sr_ronex_msgs::BoolArray());
      d_out_ptr->data.clear();
      for (unsigned int i = 0; i < n_digital_outputs_; ++i)
      {
        d_out_ptr->data.push_back(msg->data.at(i));
      }
      digital_output_.set(d_out_ptr);
    }
    else
    {
      ROS_ERROR("Wrong number of digital outputs. Must be: %d", n_digital_outputs_);
    }
  }

  void CodDecodStdIo::analogOutputCommandCB(const std_msgs::UInt16MultiArrayConstPtr& msg)
  {
    if(msg->data.size() == n_analog_outputs_)
    {
      boost::shared_ptr<std_msgs::UInt16MultiArray> a_out_ptr(new std_msgs::UInt16MultiArray());
      a_out_ptr->data.clear();
      for (unsigned int i = 0; i < n_analog_outputs_; ++i)
      {
        a_out_ptr->data.push_back(msg->data.at(i));
      }
      analog_output_.set(a_out_ptr);
    }
    else
    {
      ROS_ERROR("Wrong number of analog outputs. Must be: %d", n_analog_outputs_);
    }
  }

  void CodDecodStdIo::PWMOutputCommandCB(const std_msgs::UInt16MultiArrayConstPtr& msg)
  {
    if(msg->data.size() == n_PWM_outputs_ * 2)
    {
      boost::shared_ptr<std_msgs::UInt16MultiArray> PWM_out_ptr(new std_msgs::UInt16MultiArray());
      PWM_out_ptr->data.clear();
      for (unsigned int i = 0; i < (n_PWM_outputs_ * 2); ++i)
      {
        //The even index values correspond to the PWM channel period
        //65535 value (0xFFFF)is disallowed for the period to allow the ON-time to be able to be period+1 in any case
        if(!(i & 0x0001) )
        {
          if(msg->data.at(i) == 0xFFFF)
          {
            PWM_out_ptr->data.push_back(0xFFFE);
          }
          else
          {
            PWM_out_ptr->data.push_back(msg->data.at(i));
          }
        }
        //The odd index values correspond to the PWM channel ON-time
        //Allowed values are from 0 to (Period+1)
        else
        {
          if(msg->data.at(i) > (PWM_out_ptr->data.at(i - 1) + 1))
          {
            PWM_out_ptr->data.push_back(PWM_out_ptr->data.at(i - 1) + 1);
          }
          else
          {
            PWM_out_ptr->data.push_back(msg->data.at(i));
          }
        }
      }
      PWM_output_.set(PWM_out_ptr);
    }
    else
    {
      ROS_ERROR("Wrong number of PWM outputs. Must be: %d. Remember that you need 2 UINT values for every output (Period, ON-time)", n_PWM_outputs_);
    }
  }

  void CodDecodStdIo::add_diagnostics(std::vector<diagnostic_msgs::DiagnosticStatus> &vec,
                                 diagnostic_updater::DiagnosticStatusWrapper &d)
  {

  }

  void CodDecodStdIo::setPinAsDigitalInput(sr_cod_decod_std_io::DigitalIo pin)
  {
    //Read the digital outputs from the digital_output_ realtime box and set the right values to the digital_output_ realtime box again
    boost::shared_ptr<sr_ronex_msgs::BoolArray> d_out_ptr(new sr_ronex_msgs::BoolArray());
    boost::shared_ptr<sr_ronex_msgs::BoolArray> current_d_out_ptr;
    digital_output_.get(current_d_out_ptr);
    d_out_ptr->data = current_d_out_ptr->data;
    //set the pin as digital input
    d_out_ptr->data.at(pin * 2) = true;
    digital_output_.set(d_out_ptr);
  }

  bool CodDecodStdIo::digitalInputToBool(sr_cod_decod_std_io::DigitalIo input_pin)
  {
    //The following line gets the digital input input_pin value
    return static_cast<bool>(d_in_.data.at(input_pin));
  }

  void CodDecodStdIo::digitalInputToBool(sr_cod_decod_std_io::DigitalIo input_pin, bool *where_to_store_it)
  {
    //The following line connects the digital input input_pin to the variable pointed by where_to_store_it (it should point to a field in the HW interface)
    *where_to_store_it = static_cast<bool>(d_in_.data.at(input_pin));
  }

  double CodDecodStdIo::analogInputToDouble(sr_cod_decod_std_io::AnalogInput input_pin)
  {
    //The following line gets the analog input input_pin value
    return static_cast<double>(a_in_.data.at(input_pin));
  }

  void CodDecodStdIo::analogInputToDouble(sr_cod_decod_std_io::AnalogInput input_pin, double *where_to_store_it)
  {
    //The following line connects the analog input input_pin to the variable pointed by where_to_store_it (it should point to a field in the HW interface)
    *where_to_store_it = static_cast<double>(a_in_.data.at(input_pin));
  }

  uint16_t CodDecodStdIo::analogInputToUint16(sr_cod_decod_std_io::AnalogInput input_pin)
  {
    //The following line gets the analog input input_pin value
    return a_in_.data.at(input_pin);
  }

  void CodDecodStdIo::analogInputToUint16(sr_cod_decod_std_io::AnalogInput input_pin, uint16_t *where_to_store_it)
  {
    //The following line connects the analog input input_pin to the variable pointed by where_to_store_it (it should point to a field in the HW interface)
    *where_to_store_it = a_in_.data.at(input_pin);
  }

  int CodDecodStdIo::analogInputToInt(sr_cod_decod_std_io::AnalogInput input_pin)
  {
    //The following line gets the analog input input_pin value
    return static_cast<int>(a_in_.data.at(input_pin));
  }

  void CodDecodStdIo::analogInputToInt(sr_cod_decod_std_io::AnalogInput input_pin, int *where_to_store_it)
  {
    //The following line connects the analog input input_pin to the variable pointed by where_to_store_it (it should point to a field in the HW interface)
    *where_to_store_it = static_cast<int>(a_in_.data.at(input_pin));
  }

  void CodDecodStdIo::signToDigitalOutput(sr_cod_decod_std_io::DigitalIo output_pin, double value)
  {
    boolToDigitalOutput(output_pin, (value < 0.0));
  }

  void CodDecodStdIo::boolToDigitalOutput(sr_cod_decod_std_io::DigitalIo output_pin, bool value)
  {
    //Read the digital outputs from the digital_output_ realtime box and set the right values to the digital_output_ realtime box again
    boost::shared_ptr<sr_ronex_msgs::BoolArray> d_out_ptr(new sr_ronex_msgs::BoolArray());
    boost::shared_ptr<sr_ronex_msgs::BoolArray> current_d_out_ptr;
    digital_output_.get(current_d_out_ptr);
    d_out_ptr->data = current_d_out_ptr->data;
    //set the pin as digital output
    d_out_ptr->data.at(output_pin * 2) = false;
    //Set the dig. out. to the corresponding state
    d_out_ptr->data.at(output_pin * 2 + 1) = value;
    digital_output_.set(d_out_ptr);
  }

  void CodDecodStdIo::doubleToAnalogOutput(sr_cod_decod_std_io::AnalogOutput output_pin, double value)
  {
    uint16ToAnalogOutput(output_pin, static_cast<uint16_t>(abs(value)));
  }

  void CodDecodStdIo::uint16ToAnalogOutput(sr_cod_decod_std_io::AnalogOutput output_pin, uint16_t value)
  {
    //Read the PWM outputs from the PWM_output_ realtime box and set the right values to the PWM_output_ realtime box again
    boost::shared_ptr<std_msgs::UInt16MultiArray> analog_out_ptr(new std_msgs::UInt16MultiArray());
    boost::shared_ptr<std_msgs::UInt16MultiArray> current_analog_out_ptr;
    analog_output_.get(current_analog_out_ptr);
    analog_out_ptr->data = current_analog_out_ptr->data;
    //set the value of the output
    analog_out_ptr->data.at(output_pin) = value;
    analog_output_.set(analog_out_ptr);
  }

  void CodDecodStdIo::doubleToPWMOutput(sr_cod_decod_std_io::DigitalIo output_pin, uint16_t PWM_period, double PWM_ON_time)
  {
    uint16ToPWMOutput(output_pin, PWM_period, static_cast<uint16_t>(abs(PWM_ON_time) > static_cast<double>(PWM_period + 1) ? static_cast<double>(PWM_period + 1) : abs(PWM_ON_time) + 0.5));
  }

  void CodDecodStdIo::uint16ToPWMOutput(sr_cod_decod_std_io::DigitalIo output_pin, uint16_t PWM_period, uint16_t PWM_ON_time)
  {
    //Read the PWM outputs from the PWM_output_ realtime box and set the right values to the PWM_output_ realtime box again
    boost::shared_ptr<std_msgs::UInt16MultiArray> PWM_out_ptr(new std_msgs::UInt16MultiArray());
    boost::shared_ptr<std_msgs::UInt16MultiArray> current_PWM_out_ptr;
    PWM_output_.get(current_PWM_out_ptr);
    PWM_out_ptr->data = current_PWM_out_ptr->data;
    //set the period to PWM_period
    PWM_out_ptr->data.at(output_pin * 2) = PWM_period;
    //set the ON-time (maximum is period + 1)
    PWM_out_ptr->data.at(output_pin * 2 + 1) = static_cast<uint16_t>(PWM_ON_time > (PWM_period + 1) ? (PWM_period + 1) : PWM_ON_time);
    PWM_output_.set(PWM_out_ptr);
  }

}
