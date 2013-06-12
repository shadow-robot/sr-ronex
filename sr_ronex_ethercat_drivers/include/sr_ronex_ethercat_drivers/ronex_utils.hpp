/**
 * @file   ronex_utils.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Wed Jun 12 09:28:43 2013
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

namespace ronex
{
  static inline std::string build_name(EtherCAT_SlaveHandler *sh)
  {
    std::stringstream name, product_id;

    name << "ronex_";

    //lookup the product id in the list of human readable product ids
    product_id << sh->get_product_code();
    if(false)
    {
      //@todo read human name from param server
    }
    else
    {
      //we didn't find a human readable name for the product id, simply using the id
      name << product_id.str();
    }
    name << "_";

    //add the serial number for unique identifier
    name << sh->get_serial();

    return name.str();
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_UTILS_HPP_ */

