/**
 * @file   sr_board_mk2_gio.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Tue Jun 11 11:56:43 2013
 *
 * Copyright 2012 Shadow Robot Company Ltd.
 *
 * This program is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation, either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @brief Driver for the RoNeX mk2 General I/O module.
 *
 */

#include <sr_ronex_ethercat_drivers/sr_board_mk2_gio.hpp>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <math.h>

#include "sr_ronex_ethercat_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrBoardMk2GIO, EthercatDevice);

SrBoardMk2GIO::SrBoardMk2GIO() :
  EthercatDevice(), node_("~"), cycle_count_(0), has_stacker_(false)
{
}

SrBoardMk2GIO::~SrBoardMk2GIO()
{
  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}

void SrBoardMk2GIO::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  device_name_ = ronex::build_name( sh );
  serial_number_ = ronex::get_serial_number( sh );

  EthercatDevice::construct(sh,start_address);
  sh->set_fmmu_config( new EtherCAT_FMMU_Config(0) );
  sh->set_pd_config( new EtherCAT_PD_Config(0) );

  command_base_ = start_address;
  command_size_ = sizeof(RONEX_COMMAND_0000000C);

  start_address += command_size_;

  status_base_ = start_address;
  status_size_ = sizeof(RONEX_STATUS_0000000C);

  start_address += status_size_;

  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the board
  //
  ROS_INFO("First FMMU (command) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", command_base_, command_size_,
           static_cast<int>(COMMAND_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,                                                  // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                                                           // Logical Start Bit
                                      0x07,                                                           // Logical End Bit
                                      COMMAND_ADDRESS,                                  // Physical Start Address   (in ET1200 address space?)
                                      0x00,                                                           // Physical Start Bit
                                      false,                                                          // Read Enable
                                      true,                                                           // Write Enable
                                      true                                                            // Channel Enable
    );


  // WARNING!!!
  // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode EC_QUEUED, the so called mailbox mode)

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the board
  //
  ROS_INFO("Second FMMU (status) : start_address : 0x%08X ; size : %3d bytes ; phy addr : 0x%08X", status_base_, status_size_,
           static_cast<int>(COMMAND_ADDRESS + command_size_) );
  EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      COMMAND_ADDRESS + (command_size_ * 4),
                                      0x00,
                                      true,
                                      false,
                                      true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

  (*pd)[0] = EC_SyncMan(COMMAND_ADDRESS,                             command_size_,    EC_BUFFERED, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(COMMAND_ADDRESS + (command_size_ * 4),              status_size_,     EC_BUFFERED);


  (*pd)[0].ChannelEnable = true;
  (*pd)[0].ALEventEnable = true;
  (*pd)[0].WriteEvent    = true;

  (*pd)[1].ChannelEnable = true;

  sh->set_pd_config(pd);

  ROS_INFO("status_size_ : %d ; command_size_ : %d", status_size_, command_size_);

  ROS_INFO("Finished constructing the SrBoardMk2GIO driver");
}

int SrBoardMk2GIO::initialize(pr2_hardware_interface::HardwareInterface *hw, bool allow_unprogrammed)
{
  digital_commands_ = 0;
  ROS_INFO("Device #%02d: Product code: %u (%#010X) , Serial #: %u (%#010X)",
            sh_->get_ring_position(),
            sh_->get_product_code(),
            sh_->get_product_code(),
            sh_->get_serial(),
            sh_->get_serial());

  device_offset_ = sh_->get_ring_position();// - hand_->getBridgeRingPosition();

  return 0;
}

int SrBoardMk2GIO::readData(EthercatCom *com, EC_UINT address, void *data, EC_UINT length)
{
  return EthercatDevice::readData(com, address, data, length, FIXED_ADDR);
}


int SrBoardMk2GIO::writeData(EthercatCom *com, EC_UINT address, void const *data, EC_UINT length)
{
  return EthercatDevice::writeData(com, sh_, address, data, length, FIXED_ADDR);
}

void SrBoardMk2GIO::packCommand(unsigned char *buffer, bool halt, bool reset)
{
  RONEX_COMMAND_0000000C* command = (RONEX_COMMAND_0000000C*)(buffer);
  command->digital_out = static_cast<int32u>(digital_commands_);

  if( cycle_count_ >= 9)
  {
    ROS_DEBUG_STREAM("sending command: " << digital_commands_ << " ("<< sizeof(*command) <<")");
  }
}

bool SrBoardMk2GIO::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_0000000C* status_data = (RONEX_STATUS_0000000C *)(this_buffer+  command_size_);

  if( analogue_publishers_.size() == 0)
  {
    size_t nb_analogue_pub;
    size_t nb_digital_io;
    //The publishers haven't been initialised yet.
    // Checking if the stacker board is plugged in or not
    // to determine the number of publishers.
    if (status_data->digital_in & RONEX_0000000C_STACKER_0_PRESENT)
    {
      has_stacker_ = true;
      nb_analogue_pub = NUM_ANALOGUE_INPUTS;
      nb_digital_io = NUM_DIGITAL_IO;
    }
    else
    {
      has_stacker_ = false;
      nb_analogue_pub = NUM_ANALOGUE_INPUTS / 2;
      nb_digital_io = NUM_DIGITAL_IO / 2;
    }

    std::stringstream pub_topic;
    std::stringstream sub_topic;
    for(size_t i=0; i < nb_analogue_pub; ++i)
    {
      pub_topic.str("");
      pub_topic << device_name_ << "/state/analogue/" << i;
      analogue_publishers_.push_back(new realtime_tools::RealtimePublisher<std_msgs::UInt16>(node_, pub_topic.str(), 1));
    }

    for(size_t i=0; i < nb_digital_io; ++i)
    {
      pub_topic.str("");
      pub_topic << device_name_ << "/state/digital/" << i;
      digital_publishers_.push_back(new realtime_tools::RealtimePublisher<std_msgs::Bool>(node_, pub_topic.str(), 1));

      //adding the subscribers for receiving commands
      sub_topic.str("");
      sub_topic << device_name_ << "/command/digital/" << i;
      digital_subscribers_.push_back(node_.subscribe<std_msgs::Bool>(sub_topic.str(), 1, boost::bind(&SrBoardMk2GIO::digital_commands_cb, this, _1,  i )));
    }
  }

  if( cycle_count_ >= 9)
  {
    ROS_DEBUG_STREAM("Status size: " << sizeof(*status_data) << " " << sizeof(RONEX_COMMAND_0000000C) << "=" << sizeof(RONEX_COMMAND_0000000C_PWM) << "*" << NUM_PWM_MODULES << "+" << sizeof(int32u) << "+" << sizeof(int16u));

    for(size_t i = 0; i < analogue_publishers_.size(); ++i)
    {
      if( analogue_publishers_[i].trylock() )
      {
        analogue_msg_.data = status_data->analogue_in[i];
        analogue_publishers_[i].msg_ = analogue_msg_;
        analogue_publishers_[i].unlockAndPublish();
      }
    }

    for(size_t i = 0; i < digital_publishers_.size(); ++i)
    {
      if( digital_publishers_[i].trylock() )
      {
        digital_msg_.data = ronex::check_bit(status_data->digital_in, i);
        digital_publishers_[i].msg_ = digital_msg_;
        digital_publishers_[i].unlockAndPublish();
      }
    }

    cycle_count_ = 0;
  }
  ++cycle_count_;

  return true;
}

void SrBoardMk2GIO::digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index)
{
  ROS_ERROR_STREAM("index = " << index);
  digital_commands_ = ronex::set_bit(digital_commands_, index*2, 1);
  digital_commands_ = ronex::set_bit(digital_commands_, index*2+1, msg->data);
}

void SrBoardMk2GIO::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d, unsigned char *buffer)
{
  d.name = device_name_;
  d.summary(d.OK, "OK");
  d.hardware_id = serial_number_;

  d.clear();
  if(has_stacker_)
    d.addf("Stacker Board", "True");
  else
    d.addf("Stacker Board", "False");
}



/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
