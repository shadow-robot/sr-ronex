/**
 * @file   ronex_mapping.cpp
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
*
 * @brief  Contains the data mapping one element of a RoNeX to a joint element.
 *
 *
 */

#include "sr_ronex_mechanism_model/ronex_mapping.hpp"
#include <pr2_mechanism_model/robot.h>

namespace ronex
{
  RonexMapping::RonexMapping(TiXmlElement* ronex_el)
  {
/*  const char *ronex_name = ronex_el ? ronex_el->Attribute("name") : NULL;
    if (!ronex_name)
    {
    ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
    return false;
    }

    general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );
    if(!general_io_)
    {
    ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
    return false;
    }

    //read ronex pin from urdf
    const char *ronex_pin = ronex_el ? ronex_el->Attribute("analogue_pin") : NULL;
    if (!ronex_pin)
    {
    ROS_ERROR("RonexTransmission transmission did not specify the ronex pin.");
    return false;
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

    //read joint name
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
*/
  }

  RonexMapping::RonexMapping(TiXmlElement* ronex_el, pr2_mechanism_model::Robot* robot)
  {
    ROS_ERROR_STREAM("New element");
  }
} //end namespace

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

