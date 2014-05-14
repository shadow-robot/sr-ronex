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
 * @file   cod_decod_manager.cpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  An instance of this class will be created by the driver of every ethercat slave.
 *         Depending on the configuration read from the parameter server 
 *         (defined in cod_decod_config.yaml) It will instantiate an object of the
 *         corresponding subclass of CodDecod. The update and build_command functions will
 *         call the corresponding functions of the CodDecod object.
 **/

#include "sr_ronex_drivers/cod_decod/cod_decod_manager.hpp"
#include "sr_ronex_drivers/cod_decod/cod_decod_std_io.hpp"
#include <boost/foreach.hpp>
#include <boost/regex.hpp>

namespace sr_cod_decod
{

  CodDecodManager::CodDecodManager(hardware_interface::HardwareInterface *hw, EtherCAT_SlaveHandler *sh, int n_digital_outputs, int n_analog_outputs, int n_digital_inputs, int n_analog_inputs, int n_PWM_outputs)
    :cod_decod_loader_("sr_ronex_drivers", "sr_cod_decod::CodDecod")
  {
    uint32_t product_code = sh->get_product_code();
    uint32_t serial = sh->get_serial();
    uint32_t revision = sh->get_revision();
    uint32_t slave = sh->get_station_address()-1;

    // The point of this code to find a cod_decod class whose name matches the EtherCAT product ID and Serial Number
    // for a given device.
    // Thus cod_decod plugins would register themselves with PLUGINLIB_EXPORT_CLASS
    //
    //   PLUGINLIB_EXPORT_CLASS(class_namespace::class_type, base_class_namespace::base_class_type)
    //
    // For the CodDecodExample cod_decod example module to be use with e.g productID = 87032868 and serialNumber = 21, this statement would look something like:
    //
    //  PLUGINLIB_EXPORT_CLASS(sr_cod_decod::CodDecodExample, sr_cod_decod::CodDecod);
    //
    //
    // Unfortunately, we don't know which ROS package a particular cod_decod class is defined in.
    // To account for this, we search through the list of class names, one-by-one and find string where
    // last part of string matches product ID and serial number of a device.
    stringstream class_name_regex_str;
    class_name_regex_str << "(.*/)?" << product_code << "_" << serial;
    boost::regex class_name_regex(class_name_regex_str.str(), boost::regex::extended);

    std::vector<std::string> classes = cod_decod_loader_.getDeclaredClasses();
    std::string matching_class_name;

    BOOST_FOREACH(const std::string &class_name, classes)
    {
      if (regex_match(class_name, class_name_regex))
      {
        if (matching_class_name.size() != 0)
        {
          ROS_ERROR("Found more than 1 CodDecod class for device with product code : %u (0x%X) and serial number : %u (0x%X)", product_code, product_code, serial, serial);
          ROS_ERROR("First class name = '%s'.  Second class name = '%s'",
                    class_name.c_str(), matching_class_name.c_str());
        }
        matching_class_name = class_name;
      }
    }

    if (matching_class_name.size() != 0)
    {
      //ROS_WARN("Using driver class '%s' for device with product code %d",
      //         matching_class_name.c_str(), product_code);
      try {
        cod_decod_ = cod_decod_loader_.createInstance( matching_class_name );
      }
      catch (pluginlib::LibraryLoadException &e)
      {
        cod_decod_.reset();
        ROS_FATAL("Unable to load CodDecod plugin for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                  slave, product_code, product_code, serial, serial, revision, revision);
        ROS_FATAL("%s", e.what());
      }
    }
    else
    {
      ROS_INFO("Unable to find a dedicated CodDecod plugin for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                slave, product_code, product_code, serial, serial, revision, revision);
      ROS_INFO("Possible classes:");
      BOOST_FOREACH(const std::string &class_name, classes)
      {
        ROS_INFO("  %s", class_name.c_str());
      }
      ROS_INFO("Loading the default CodDecod plugin: CodDecodStdIo");
      try {
        cod_decod_ = cod_decod_loader_.createInstance( "sr_ronex_drivers/87032868_0" );
      }
      catch (pluginlib::LibraryLoadException &e)
      {
        cod_decod_.reset();
        ROS_FATAL("Unable to load CodDecod plugin for slave #%d, product code: %u (0x%X), serial: %u (0x%X), revision: %d (0x%X)",
                  slave, product_code, product_code, serial, serial, revision, revision);
        ROS_FATAL("%s", e.what());
      }
    }

    if (cod_decod_ != NULL)
    {
      cod_decod_->construct(hw, sh, n_digital_outputs, n_analog_outputs, n_digital_inputs, n_analog_inputs, n_PWM_outputs);
    }

  }

  CodDecodManager::~CodDecodManager()
  {
  }

  void CodDecodManager::update(unsigned char *status_buffer)
  {
    if(cod_decod_)
    {
      cod_decod_->update(status_buffer);
    }
  }

  void CodDecodManager::build_command(unsigned char *command_buffer)
  {
    if(cod_decod_)
    {
      cod_decod_->build_command(command_buffer);
    }
  }

}
