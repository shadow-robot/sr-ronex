/**
 * @file   analogue_to_effort.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
* Copyright 2011 Shadow Robot Company Ltd.
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
 * @brief  Contains the data mapping one pin of a GIO module to the effort of the joint.
 *
 *
 */

#include "sr_ronex_mechanism_model/mapping/general_io/analogue_to_effort.hpp"
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
          //@todo calibrate here
          js[0]->measured_effort_ = general_io_->state_.analogue_[pin_index_];
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
