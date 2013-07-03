/**
 * @file   position.cpp
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
 * @brief  Contains the data mapping one pin of a GIO module to the position of the joint.
 *
 *
 */

#include "sr_ronex_mechanism_model/mapping/general_io/analogue_to_position.hpp"
#include <pr2_mechanism_model/robot.h>
#include <boost/lexical_cast.hpp>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      AnalogueToPosition::AnalogueToPosition(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot)
      {
        ROS_ERROR_STREAM("ADDING Analogue to position");

        const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
        if (!ronex_name)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
          return;
        }

        general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );
        if(!general_io_)
        {
          ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
          return;
        }

        //read ronex pin from urdf
        const char *ronex_pin = mapping_el ? mapping_el->Attribute("analogue_pin") : NULL;
        if (!ronex_pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the ronex pin.");
          return;
        }
        //convert pin to size_t and check it's in the correct bounds
        try
        {
          pin_index_ = boost::lexical_cast<size_t>( ronex_pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pin to an int.");
        }
      }

      AnalogueToPosition::~AnalogueToPosition()
      {
      }

      void AnalogueToPosition::propagateFromRonex(std::vector<pr2_mechanism_model::JointState*>& js)
      {
        assert(js.size() == 1);

        //we have to check here for the size otherwise the general io hasn't been updated.
        if( pin_out_of_bound_ )
        {
          if( pin_index_ >= general_io_->state_.analogue_.size() )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified pin is out of bound: " << pin_index_ << " / max = " << general_io_->state_.analogue_.size() << ", not propagating the RoNeX data to the joint position.");
            return;
          }
        }
        //@todo calibrate here?
        js[0]->position_ = general_io_->state_.analogue_[pin_index_];
      }

      void AnalogueToPosition::propagateToRonex(std::vector<pr2_mechanism_model::JointState*>& js)
      {}
    }
  }
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
