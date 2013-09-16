/**
 * @file   analogue_to_joint_position.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Jul  1 08:09:07 2013
 *
* Copyright 2013 Shadow Robot Company Ltd.
*
* This program is Proprietary software: you cannot redistribute it or modify it
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.
*
*
 * @brief  A transmission for mapping one analogue input pin from a general I/O
 *         RoNeX module to the position of the joint.
 *
 *
 */

#include <pr2_mechanism_model/robot.h>
#include <sr_ronex_transmissions/ronex_transmission.hpp>
#include "pluginlib/class_list_macros.h"
#include <cstring>

#include "sr_ronex_transmissions/mapping/general_io/analogue_to_position.hpp"
#include "sr_ronex_transmissions/mapping/general_io/analogue_to_effort.hpp"
#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm.hpp"

PLUGINLIB_EXPORT_CLASS( ronex::RonexTransmission, pr2_mechanism_model::Transmission)

namespace ronex
{
  bool RonexTransmission::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";


    //reading the joint name
    TiXmlElement *jel = elt->FirstChildElement("joint");
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("RonexTransmission did not specify joint name");
      return false;
    }

    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
    if (!joint)
    {
      ROS_ERROR("RonexTransmission could not find joint named \"%s\"", joint_name);
      return false;
    }
    joint_names_.push_back(joint_name);

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
          ROS_WARN_STREAM("Property not recognised: " << property);
      }
    }

    return true;
  }

  bool RonexTransmission::initXml(TiXmlElement *elt)
  {
    //not doing anything (used in simulation only)

    return true;
  }

  void RonexTransmission::propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                                            std::vector<pr2_mechanism_model::JointState*>& js)
  {
    for(ronex_iter_ = ronex_mappings_.begin(); ronex_iter_ != ronex_mappings_.end(); ++ronex_iter_)
    {
      ronex_iter_->propagateFromRonex(js);
    }
  }

  void RonexTransmission::propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>& js,
                                                     std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    //not doing anything (used in simulation)
  }

  void RonexTransmission::propagateEffort(std::vector<pr2_mechanism_model::JointState*>& js,
                                          std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    for(ronex_iter_ = ronex_mappings_.begin(); ronex_iter_ != ronex_mappings_.end(); ++ronex_iter_)
    {
      ronex_iter_->propagateToRonex(js);
    }
  }

  void RonexTransmission::propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>& js,
                                                   std::vector<pr2_mechanism_model::JointState*>& as)
  {
    //not doing anything (used in simulation)
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
