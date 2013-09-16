/**
 * @file   analogue_to_position.cpp
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
        : pin_out_of_bound_(true)
      {
        const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
        if (!ronex_name)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
          return;
        }

        //@todo: when we have multiple available module types check the module type when casting
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
        //convert pin to size_t
        try
        {
          pin_index_ = boost::lexical_cast<size_t>( ronex_pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pin to an int.");
          return;
        }

        //read scale
        const char *scale = mapping_el ? mapping_el->Attribute("scale") : NULL;
        if (!ronex_pin)
        {
          ROS_WARN("RonexTransmission transmission did not specify the scale, using 1.0.");
          scale = "1.0";
        }
        //convert scale to double
        try
        {
          scale_ = boost::lexical_cast<double>( scale );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_WARN("RonexTransmission: Couldn't parse scale to a double, using 1.0.");
          scale_ = 1.0;
        }

        //read offset
        const char *offset = mapping_el ? mapping_el->Attribute("offset") : NULL;
        if (!ronex_pin)
        {
          ROS_WARN("RonexTransmission transmission did not specify the offset, using 0.0.");
          offset = "0.0";
        }
        //convert offset to double
        try
        {
          offset_ = boost::lexical_cast<double>( offset );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_WARN("RonexTransmission: Couldn't parse offset to a double, using 0.0.");
          offset_ = 0.0;
        }
      }

      AnalogueToPosition::~AnalogueToPosition()
      {
      }

      void AnalogueToPosition::propagateFromRonex(std::vector<pr2_mechanism_model::JointState*>& js)
      {
        assert(js.size() == 1);

        if( check_pin_in_bound_() )
        {
          js[0]->position_ = compute_scaled_data_();
        }
      }

      bool AnalogueToPosition::check_pin_in_bound_()
      {
        //we have to check here for the size otherwise the general io hasn't been updated.
        if( pin_out_of_bound_ )
        {
          if( pin_index_ >= general_io_->state_.analogue_.size() )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified pin is out of bound: " << pin_index_ << " / max = " << general_io_->state_.analogue_.size() << ", not propagating the RoNeX data to the joint position.");

            pin_out_of_bound_ = true;
            return false;
          }
        }

        pin_out_of_bound_ = false;
        return true;
      }

      double AnalogueToPosition::compute_scaled_data_()
      {
        return general_io_->state_.analogue_[pin_index_]*scale_ + offset_;
      }
    }
  }
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
