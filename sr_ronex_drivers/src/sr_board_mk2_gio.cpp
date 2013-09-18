/**
 * @file   sr_board_mk2_gio.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
 * Copyright 2013 Shadow Robot Company Ltd.
*
* This program is Proprietary software: you cannot redistribute it or modify it
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.
*
 *
 * @brief Driver for the RoNeX mk2 General I/O module.
 *
 */

#include <sr_ronex_drivers/sr_board_mk2_gio.hpp>

#include <dll/ethercat_dll.h>
#include <al/ethercat_AL.h>
#include <dll/ethercat_device_addressed_telegram.h>
#include <dll/ethercat_frame.h>

#include <sstream>
#include <iomanip>
#include <boost/foreach.hpp>
#include <math.h>

#include "sr_ronex_drivers/ronex_utils.hpp"

PLUGINLIB_EXPORT_CLASS(SrBoardMk2GIO, EthercatDevice);

const std::string SrBoardMk2GIO::product_alias_ = "general_io";

SrBoardMk2GIO::SrBoardMk2GIO() :
  EthercatDevice(), node_("~"), cycle_count_(0), has_stacker_(false), input_mode_(false)
{
}

SrBoardMk2GIO::~SrBoardMk2GIO()
{
  //remove parameters from server
  std::stringstream param_path;
  param_path << "/ronex/" << parameter_id_ ;
  ros::param::del(param_path.str());


  delete sh_->get_fmmu_config();
  delete sh_->get_pd_config();
}

void SrBoardMk2GIO::construct(EtherCAT_SlaveHandler *sh, int &start_address)
{
  //get the alias from the parameter server if it exists
  std::string path_to_alias, alias;
  path_to_alias = "/ronexes/mapping/" + serial_number_;
  if( ros::param::get(path_to_alias, alias))
  {
    ronex_id_ = alias;
  }
  else
  {
    //no alias found, using the serial number directly.
    ronex_id_ = serial_number_ ;
  }

  device_name_ = ronex::build_name( product_alias_, ronex_id_ );
  serial_number_ = ronex::get_serial_number( sh );

  EthercatDevice::construct(sh,start_address);
  sh->set_fmmu_config( new EtherCAT_FMMU_Config(0) );
  sh->set_pd_config( new EtherCAT_PD_Config(0) );

  command_base_  = start_address;
  command_size_  = COMMAND_ARRAY_SIZE_BYTES;

  start_address += command_size_;
  status_base_   = start_address;
  status_size_   = STATUS_ARRAY_SIZE_BYTES;

  start_address += status_size_;

  // ETHERCAT_COMMAND_DATA
  //
  // This is for data going TO the board
  //

  if ( (PROTOCOL_TYPE) == (EC_BUFFERED) )
  {
    ROS_INFO("Using EC_BUFFERED");
  }
  else if ( (PROTOCOL_TYPE) == (EC_QUEUED) )
  {
    ROS_INFO("Using EC_QUEUED");
  }

  ROS_INFO("First FMMU (command) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", command_base_, command_size_,
           static_cast<int>(COMMAND_ADDRESS) );
  EC_FMMU *commandFMMU = new EC_FMMU( command_base_,            // Logical Start Address    (in ROS address space?)
                                      command_size_,
                                      0x00,                   // Logical Start Bit
                                      0x07,                   // Logical End Bit
                                      COMMAND_ADDRESS,        // Physical Start Address   (in ET1200 address space?)
                                      0x00,                   // Physical Start Bit
                                      false,                   // Read Enable
                                      true,                    // Write Enable
                                      true                     // Channel Enable
    );


  // WARNING!!!
  // We are leaving (command_size_ * 4) bytes in the physical memory of the device, but strictly we only need to
  // leave (command_size_ * 3). This change should be done in the firmware as well, otherwise it won't work.
  // This triple buffer is needed in the ethercat devices to work in EC_BUFFERED mode (in opposition to the other mode EC_QUEUED, the so called mailbox mode)

  // ETHERCAT_STATUS_DATA
  //
  // This is for data coming FROM the board
  //
  ROS_INFO("Second FMMU (status) : Logical address: 0x%08X ; size: %3d bytes ; ET1200 address: 0x%08X", status_base_, status_size_,
           static_cast<int>(STATUS_ADDRESS) );
  EC_FMMU *statusFMMU = new EC_FMMU(  status_base_,
                                      status_size_,
                                      0x00,
                                      0x07,
                                      STATUS_ADDRESS,
                                      0x00,
                                      true,
                                      false,
                                      true);


  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(2);

  (*fmmu)[0] = *commandFMMU;
  (*fmmu)[1] = *statusFMMU;

  sh->set_fmmu_config(fmmu);

  EtherCAT_PD_Config *pd = new EtherCAT_PD_Config(2);

// SyncMan takes the physical address
  (*pd)[0] = EC_SyncMan(COMMAND_ADDRESS,              command_size_,    PROTOCOL_TYPE, EC_WRITTEN_FROM_MASTER);
  (*pd)[1] = EC_SyncMan(STATUS_ADDRESS,               status_size_,     PROTOCOL_TYPE);


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

  //add the RoNeX to the hw interface
  general_io_.reset( new ronex::GeneralIO() );
  general_io_->name_ = device_name_;

  build_topics_();

  ROS_INFO_STREAM("Adding a general_io RoNeX module to the hardware interface: " << device_name_);
  //Using the name of the ronex to prefix the state topic

  hw->addCustomHW( general_io_.get() );

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
  RONEX_COMMAND_02000001* command = (RONEX_COMMAND_02000001*)(buffer);

  //digital command
  for (size_t i = 0; i < general_io_->command_.digital_.size(); ++i)
  {
    if (input_mode_)
    {
      // Just set the pin to input mode, gets read in the status
      ronex::set_bit(digital_commands_, i*2, 1);
    }
    else
    { // Output
      ronex::set_bit(digital_commands_, i*2, 0);
      ronex::set_bit(digital_commands_, i*2+1, general_io_->command_.digital_[i]);
    }
  }

  command->digital_out = static_cast<int32u>(digital_commands_);

  //PWM command
  for (size_t i = 0; i < general_io_->command_.pwm_.size(); ++i)
  {
    command->pwm_module[i].pwm_period = general_io_->command_.pwm_[i].period;
    command->pwm_module[i].pwm_on_time_0 = general_io_->command_.pwm_[i].on_time_0;
    command->pwm_module[i].pwm_on_time_1 = general_io_->command_.pwm_[i].on_time_1;
  }

  command->pwm_clock_divider = general_io_->command_.pwm_clock_divider_;
}

bool SrBoardMk2GIO::unpackState(unsigned char *this_buffer, unsigned char *prev_buffer)
{
  RONEX_STATUS_02000001* status_data = (RONEX_STATUS_02000001 *)(this_buffer+  command_size_);

  if( general_io_->state_.analogue_.size() == 0)
  {
    size_t nb_analogue_pub, nb_digital_io, nb_pwm_modules;
    //The publishers haven't been initialised yet.
    // Checking if the stacker board is plugged in or not
    // to determine the number of publishers.
    if (status_data->flags & RONEX_02000001_FLAGS_STACKER_0_PRESENT)
    {
      has_stacker_ = true;
      nb_analogue_pub = NUM_ANALOGUE_INPUTS;
      nb_digital_io = NUM_DIGITAL_IO;
      nb_pwm_modules = NUM_PWM_MODULES;
    }
    else
    {
      has_stacker_ = false;
      nb_analogue_pub = NUM_ANALOGUE_INPUTS / 2;
      nb_digital_io = NUM_DIGITAL_IO / 2;
      nb_pwm_modules = NUM_PWM_MODULES / 2;
    }

    //resizing the GeneralIO in the HardwareInterface
    general_io_->state_.analogue_.resize(nb_analogue_pub);
    general_io_->state_.digital_.resize(nb_digital_io);
    general_io_->command_.digital_.resize(nb_digital_io);
    general_io_->command_.pwm_.resize(nb_pwm_modules);

    //init the state message
    state_msg_.analogue.resize(nb_analogue_pub);
    state_msg_.digital.resize(nb_digital_io);

    //dynamic reconfigure server is instantiated here
    // as we need the different vectors to be initialised
    // before running the first configuration.
    dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<sr_ronex_drivers::GeneralIOConfig>(ros::NodeHandle(device_name_)));
    function_cb_ = boost::bind(&SrBoardMk2GIO::dynamic_reconfigure_cb, this, _1, _2);
    dynamic_reconfigure_server_->setCallback(function_cb_);
  } //end first time, the sizes are properly initialised, simply fill in the data

  for(size_t i = 0; i < general_io_->state_.analogue_.size(); ++i )
  {
    general_io_->state_.analogue_[i] = status_data->analogue_in[i];
  }

  for(size_t i = 0; i < general_io_->state_.digital_.size(); ++i )
  {
    general_io_->state_.digital_[i] = ronex::check_bit(status_data->digital_in, i);
  }

  //publishing at 100Hz
  if(cycle_count_ > 9)
  {
    state_msg_.header.stamp = ros::Time::now();

    //update state message
    for(size_t i=0; i < general_io_->state_.analogue_.size(); ++i)
    {
      state_msg_.analogue[i] = general_io_->state_.analogue_[i];
    }

    for(size_t i=0; i < general_io_->state_.digital_.size(); ++i)
    {
      state_msg_.digital[i] = general_io_->state_.digital_[i];
    }

    state_msg_.pwm_clock_divider = general_io_->command_.pwm_clock_divider_;
    state_msg_.input_mode = input_mode_;

    //publish
    if( state_publisher_->trylock() )
    {
      state_publisher_->msg_ = state_msg_;
      state_publisher_->unlockAndPublish();
    }

    cycle_count_ = 0;
  }

  cycle_count_++;
  return true;
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


void SrBoardMk2GIO::dynamic_reconfigure_cb(sr_ronex_drivers::GeneralIOConfig &config, uint32_t level)
{
  general_io_->command_.pwm_clock_divider_ = static_cast<int16u>(config.pwm_clock_divider);

  input_mode_ = config.input_mode;

  if( general_io_->command_.pwm_.size() > 0 )
    general_io_->command_.pwm_[0].period = static_cast<int16u>(config.pwm_period_0);
  if( general_io_->command_.pwm_.size() > 1 )
    general_io_->command_.pwm_[1].period = static_cast<int16u>(config.pwm_period_1);
  if( general_io_->command_.pwm_.size() > 2 )
    general_io_->command_.pwm_[2].period = static_cast<int16u>(config.pwm_period_2);
  if( general_io_->command_.pwm_.size() > 3 )
    general_io_->command_.pwm_[3].period = static_cast<int16u>(config.pwm_period_3);
  if( general_io_->command_.pwm_.size() > 4 )
    general_io_->command_.pwm_[4].period = static_cast<int16u>(config.pwm_period_4);
  if( general_io_->command_.pwm_.size() > 5 )
    general_io_->command_.pwm_[5].period = static_cast<int16u>(config.pwm_period_5);
}

void SrBoardMk2GIO::build_topics_()
{
  std::stringstream path;
  path << "/ronex/" << product_alias_ << "/";
  path << ronex_id_ << "/";

  //loading everything into the parameter server
  parameter_id_ = ronex::get_ronex_param_id("");
  std::stringstream param_path, tmp_param;
  param_path << "/ronex/" << parameter_id_ << "/";
  tmp_param << ronex::get_product_code(sh_);
  ros::param::set(param_path.str() + "product_id", tmp_param.str());
  ros::param::set(param_path.str() + "product_name", product_alias_);
  ros::param::set(param_path.str() + "ronex_id", ronex_id_);
  ros::param::set(param_path.str() + "path", path.str());
  ros::param::set(param_path.str() + "serial", serial_number_);

  //Advertising the realtime state publisher
  path << "state";
  state_publisher_.reset(new realtime_tools::RealtimePublisher<sr_ronex_msgs::GeneralIOState>(node_, path.str(), 1));
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
