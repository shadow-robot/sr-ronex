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
 * @file   sr_ronex_utilities.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Very light, header only library containing a
 *         set of useful functions for the RoNeX drivers.
 **/

#pragma once

#include <sstream>
#include <bitset>
#include <ros/ros.h>
#include <boost/lexical_cast.hpp>

namespace ronex
{
  /**
   * Checks if the bit is set for the given index.
   *
   * @param data The var containing the different bits.
   * @param index The index for which we're checking the bit.
   *
   * @return true if bit at index is set in data.
   */
  static inline bool check_bit(uint16_t data, size_t index)
  {
    // x8 because sizeof returns size in bytes not bits
    return std::bitset<sizeof(uint16_t)*8>(data).test(index);
  }

  /**
   * Sets the given bit to the given value.
   *
   * @param data The var containing the different bits.
   * @param index The index for which we're setting the bit.
   * @param value The value we want the bit to take
   */
  static inline void set_bit(uint32_t &data, size_t index, bool value)
  {
    //x8 because sizeof returns size in bytes not bits
    std::bitset<sizeof(uint32_t)*8> tmp(data);
    tmp.set(index, value);
    data = static_cast<uint32_t>(tmp.to_ulong());
  }

   /**
   * Checks the ronexes already present on the parameter server and returns an id on which
   *  the given ronex is stored on the parameter server.
   *
   * The parameter server contains:
   *  /ronex/0/product_id = "0x200001"
   *  /ronex/0/produc_name = "general_io"
   *  /ronex/0/ronex_id = "my_beautiful_ronex" or serial if no alias
   *  /ronex/0/path = "/ronex/general_io/my_beautiful_ronex"
   *  /ronex/0/serial = "1234"
   *
   *  /ronex/1/...
   *
   * @param ronex_id Either the alias or the serial number if no alias is specified.
   *                 If empty string given, then returns the first available id.
   * @return the index of the ronex in the parameter server. -1 if not found.
   *         or the next free index if ronex_id == ""
   */
  static inline int get_ronex_param_id(std::string ronex_id)
  {
    std::string param;

    int ronex_parameter_id = 0;
    while( true )
    {
      std::stringstream ss;
      ss << "/ronex/devices/" << ronex_parameter_id << "/ronex_id";
      if(ros::param::get(ss.str(), param) )
      {
        if( ronex_id.compare("") != 0 )
        {
          if( ronex_id.compare(param) == 0)
          {
            return ronex_parameter_id;
          }
        }
        ++ronex_parameter_id;
      }
      else
      {
        if( ronex_id.compare("") != 0)
        {
          //we were looking for a specific ronex and didn't find it -> return -1
          return -1;
        }
        return ronex_parameter_id;
      }
    }

    return -1;
  } 
 
  /**
   * Construct a string for e.g., ros::param::get as the key.
   *
   * @param ronex_parameter_id Part of the key (e.g., 2).
   * @param part Part of the key (e.g., "product_name").
   * @return The key (e.g., "/ronex/devices/2/product_name").
   **/
   static inline std::string get_ronex_devices_string(int ronex_parameter_id, std::string part)
   {
     std::string key("/ronex/devices/");
     key += boost::lexical_cast<std::string>(ronex_parameter_id);
     key += "/";
     key += part;
     return key;
   }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

