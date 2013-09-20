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
 * @file   ronex_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A set of useful functions for the RoNeX drivers.
 *         This library is relying on the ethercat_hardware package
 *         that's why we keep it separate from the sr_ronex_utilities
 *         library (to keep sr_ronex_utilities really light).
 **/

#ifndef _RONEX_UTILS_HPP_
#define _RONEX_UTILS_HPP_

#include <ethercat_hardware/ethercat_device.h>
#include <sstream>
#include <bitset>
#include <ros/ros.h>

#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

namespace ronex
{
  static inline std::string get_serial_number(EtherCAT_SlaveHandler *sh)
  {
    std::stringstream id;
    id << sh->get_serial();
    return id.str();
  }

  static inline std::string get_product_code(EtherCAT_SlaveHandler *sh)
  {
    std::stringstream id;
    id << sh->get_product_code();
    return id.str();
  }

  /**
   * Building the name of the RoNeX, to be stored in the CustomHW map
   *  of the Hardware Interface. The name used is /ronex/product_name/SERIAL_NUMBER
   *
   * example: /ronex/general_io/1234
   *
   * @param sh the EtherCAT SlaveHandler (used to read the serial number)
   * @param product_alias the human readable name for this RoNeX module
   * @param ronex_id the unique id for the ronex (serial or alias)
   *
   * @return a name to be used in the CustomHW map.
   */
  static inline std::string build_name( const std::string product_alias,
                                        std::string ronex_id)
  {
    std::stringstream name;
    name << "/ronex/" << product_alias << "/" << ronex_id;

    return name.str();
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_UTILS_HPP_ */

