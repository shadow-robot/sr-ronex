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
 * @file   analogue_to_effort.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Contains the data mapping one pin of a GIO module to the effort of the joint.
 **/

#include "sr_ronex_transmissions/mapping/general_io/analogue_to_effort.hpp"
#include <ros_ethercat_model/robot_state.hpp>
#include <boost/lexical_cast.hpp>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      AnalogueToEffort::AnalogueToEffort(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot)
        : AnalogueToPosition(mapping_el, robot)
      {
      }

      AnalogueToEffort::~AnalogueToEffort()
      {
      }

      void AnalogueToEffort::propagateFromRonex(std::vector<ros_ethercat_model::JointState*>& js)
      {
        assert(js.size() == 1);

        if( check_pin_in_bound_() )
        {
          js[0]->measured_effort_ = compute_scaled_data_();
        }
      }
    }
  }
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
