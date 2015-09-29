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
 * @file   sr_board_adc16.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief Driver for the RoNeX ADC16 module.
 **/

#ifndef _SR_BOARD_ADC16_HPP_
#define _SR_BOARD_ADC16_HPP_

#include <ros_ethercat_hardware/ethercat_device.h>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_msgs/ADC16State.h>

#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000008_ADC16_00.h>
#include <sr_ronex_hardware_interface/adc16_hardware_interface.hpp>

#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/scoped_ptr.hpp>
#include <vector>
#include <queue>

#include <dynamic_reconfigure/server.h>
#include "sr_ronex_drivers/ADC16Config.h"

using namespace std;

class SrBoardADC16 : public EthercatDevice
{
public:
  virtual void construct(EtherCAT_SlaveHandler *sh, int &start_address);
  virtual int initialize(hardware_interface::HardwareInterface *hw, bool allow_unprogrammed=true);

  SrBoardADC16();
  virtual ~SrBoardADC16();

  void dynamic_reconfigure_cb(sr_ronex_drivers::ADC16Config &config, uint32_t level);

protected:
  ///Replaces the product ID with a human readable product alias.
  static const std::string product_alias_;

  ///A unique identifier for the ronex (either serial number or alias if provided)
  std::string ronex_id_;

  string reason_;
  int level_;

  int command_base_;
  int status_base_;

  ros::NodeHandle node_;

  ///The ADC16 module which is added as a CustomHW to the hardware interface
  ronex::ADC16 *adc16_;

  /**
   * A counter used to publish the data at 100Hz:
   *  count 10 cycles, then reset the cycle_count to 0.
   */
  short cycle_count_;

  ///the digital commands sent at each cycle (updated when we call the topic)
  int32u digital_commands_;

  ///Name under which the RoNeX will appear (prefix the topics etc...)
  std::string device_name_;
  std::string serial_number_;

  ///Offset of device position from first device
  int device_offset_;

  //False if initial configuration has not been sent
  bool config_received_;
  //True if registers are being set
  bool reg_flag_;
  //Switch for states of setting registers
  int reg_state_;
  //Count for feedback
  int feedback_flag_;

  ///True if a stacker board is plugged in the RoNeX
  bool has_stacker_;

  ///False to run digital pins as output, True to run as input
  std::vector<bool> input_mode_;

  //Queue of commands to send to register
  std::queue<RONEX_COMMAND_02000008> command_queue_;
  std::queue<RONEX_COMMAND_02000008> queue_backup_;

  //1 for single ended adc, 2 for differential
  std::vector<int> pin_mode_;

  //Number of stacks present
  int stack;

  //values_s0: first single ended pin values,
  //values_s1: second single ended pin values,
  //values_d: differential pin values
  //fake_values: the additional bits sent
  //padded: the addition of requested and fake values
  std::vector<unsigned short int> values_s0_;
  std::vector<unsigned short int> values_s1_;
  std::vector<unsigned short int> values_d_;
  std::vector<unsigned short int> fake_values_s0_;
  std::vector<unsigned short int> fake_values_s1_;
  std::vector<unsigned short int> padded_s0_;
  std::vector<unsigned short int> padded_s1_;


  void packCommand(unsigned char *buffer, bool halt, bool reset);
  bool unpackState(unsigned char *this_buffer, unsigned char *prev_buffer);

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer);

  ///publisher for the data.
  boost::scoped_ptr<realtime_tools::RealtimePublisher<sr_ronex_msgs::ADC16State> > state_publisher_;
  ///Temporary message
  sr_ronex_msgs::ADC16State state_msg_;

  ///Dynamic reconfigure server for setting the parameters of the driver
  boost::scoped_ptr<dynamic_reconfigure::Server<sr_ronex_drivers::ADC16Config> > dynamic_reconfigure_server_;

  dynamic_reconfigure::Server<sr_ronex_drivers::ADC16Config>::CallbackType function_cb_;

  ///building the topics for publishing the state.
  void build_topics_();

  ///Id of this ronex on the parameter server
  int parameter_id_;
};

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _SR_BOARD_ADC16_HPP_ */

