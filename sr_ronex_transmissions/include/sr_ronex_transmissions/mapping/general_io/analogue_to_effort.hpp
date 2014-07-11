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
 * @file   analogue_to_effort.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Contains the data mapping one pin of a GIO module to the effort of
 *         the joint. Based on AnalogueToPosition as it's very close.
 **/

#ifndef SR_RONEX_GENERAL_IO_EFFORT_MAPPING_H_
#define SR_RONEX_GENERAL_IO_EFFORT_MAPPING_H_

#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include "sr_ronex_transmissions/mapping/general_io/analogue_to_position.hpp"

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      class AnalogueToEffort
        : public AnalogueToPosition
      {
      public:
        AnalogueToEffort(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot);

        /**
         * Propagating the specified analogue pin data to the given joint measured_effort.
         *
         * @param js joint_state of the joint specified in the transmission
         */
        virtual void propagateFromRonex(ros_ethercat_model::JointState *js);
      };
    }
  }
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* SR_RONEX_GENERAL_IO_EFFORT_MAPPING_H_ */
