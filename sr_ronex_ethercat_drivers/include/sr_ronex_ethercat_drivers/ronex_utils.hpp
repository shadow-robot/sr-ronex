/**
 * @file   ronex_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
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
 * @brief A set of useful functions for the RoNeX drivers.
 *
 *
 */

#ifndef _RONEX_UTILS_HPP_
#define _RONEX_UTILS_HPP_

#include <ethercat_hardware/ethercat_device.h>
#include <sstream>
#include <bitset>

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

  static inline std::string build_name(EtherCAT_SlaveHandler *sh)
  {
    std::stringstream name;

    name << "ronex_";

    //lookup the product id in the list of human readable product ids
    if(false)
    {
      //@todo read human name from param server
    }
    else
    {
      //we didn't find a human readable name for the product id, simply using the id
      name << get_product_code(sh);
    }
    name << "_";

    //add the serial number for unique identifier
    name << get_serial_number(sh);

    return name.str();
  }

  /**
   * Checks if the bit is set for the given index.
   *
   * @param data The var containing the different bits.
   * @param index The index for which we're checking the bit.
   *
   * @return true if bit at index is set in data.
   */
  bool check_bit(int16u data, size_t index)
  {
    // x8 because sizeof returns size in bytes not bits
    return std::bitset<sizeof(int16u)*8>(data).test(index);
  }

  /**
   * Sets the given bit to the given value.
   *
   * @param data The var containing the different bits.
   * @param index The index for which we're setting the bit.
   * @param value The value we want the bit to take
   */
  void set_bit(int32u &data, size_t index, bool value)
  {
    //*8 because sizeof returns size in bytes not bits
    std::bitset<sizeof(int32u)*8> tmp(data);
    tmp.set(index, value);
    data = static_cast<int32u>(tmp.to_ulong());
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_UTILS_HPP_ */

