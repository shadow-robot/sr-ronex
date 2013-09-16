/**
 * @file   analogue_to_effort.cpp
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
 * @brief  Contains the data mapping one pin of a GIO module to the effort of the joint.
 *
 *
 */

#include "sr_ronex_transmissions/mapping/general_io/analogue_to_effort.hpp"
#include <pr2_mechanism_model/robot.h>
#include <boost/lexical_cast.hpp>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      AnalogueToEffort::AnalogueToEffort(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot)
        : AnalogueToPosition(mapping_el, robot)
      {
      }

      AnalogueToEffort::~AnalogueToEffort()
      {
      }

      void AnalogueToEffort::propagateFromRonex(std::vector<pr2_mechanism_model::JointState*>& js)
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
