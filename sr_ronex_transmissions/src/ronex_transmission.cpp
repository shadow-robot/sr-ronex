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
 * @file   analogue_to_joint_position.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Jul  1 08:09:07 2013
 * @brief  A transmission for mapping one analogue input pin from a general I/O
 *         RoNeX module to the position of the joint.
 **/

#include <sr_ronex_transmissions/ronex_transmission.hpp>
#include "pluginlib/class_list_macros.h"
#include <cstring>

#include "sr_ronex_transmissions/mapping/general_io/analogue_to_position.hpp"
#include "sr_ronex_transmissions/mapping/general_io/analogue_to_effort.hpp"
#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm.hpp"

PLUGINLIB_EXPORT_CLASS( ronex::RonexTransmission, ros_ethercat_model::Transmission)

namespace ronex
{
  bool RonexTransmission::initXml(TiXmlElement *elt, ros_ethercat_model::RobotState *robot)
  {
    if (!ros_ethercat_model::Transmission::initXml(elt, robot))
      return false;

    //Extract all the mapping information from the transmission
    for( TiXmlElement *mapping_el = elt->FirstChildElement("mapping"); mapping_el;
         mapping_el = mapping_el->NextSiblingElement("mapping") )
    {
      const char *property = mapping_el ? mapping_el->Attribute("property") : NULL;
      if (!property)
      {
        ROS_ERROR("RonexTransmission: no property defined for the mapping. Skipping the mapping.");
      }
      else
      {
        //@todo is there a better way of instantiating the correct type?
        if( std::strcmp("position", property) == 0 )
        {
          ronex_mappings_.push_back( new mapping::general_io::AnalogueToPosition(mapping_el, robot) );
        }
        else if( std::strcmp("effort", property) == 0 )
        {
          ronex_mappings_.push_back( new mapping::general_io::AnalogueToEffort(mapping_el, robot) );
        }
        else if( std::strcmp("command", property) == 0 )
        {
          ronex_mappings_.push_back( new mapping::general_io::CommandToPWM(mapping_el, robot) );
        }
        else
          ROS_WARN_STREAM("Property not recognized: " << property);
      }
    }

    return true;
  }

  void RonexTransmission::propagatePosition()
  {
    for(ronex_iter_ = ronex_mappings_.begin(); ronex_iter_ != ronex_mappings_.end(); ++ronex_iter_)
    {
      ronex_iter_->propagateFromRonex(joint_);
    }
  }

  void RonexTransmission::propagateEffort()
  {
    for(ronex_iter_ = ronex_mappings_.begin(); ronex_iter_ != ronex_mappings_.end(); ++ronex_iter_)
    {
      ronex_iter_->propagateToRonex(joint_);
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
