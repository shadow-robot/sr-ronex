/**
 * @file   analogue_to_joint_position.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Mon Jul  1 08:09:07 2013
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
 * @brief  A transmission for mapping one analogue input pin from a general I/O
 *         RoNeX board to the position of the joint.
 *
 *
 */

#include <pr2_mechanism_model/robot.h>
#include <sr_ronex_mechanism_model/analogue_to_joint_position.hpp>
#include "pluginlib/class_list_macros.h"
#include <boost/lexical_cast.hpp>

PLUGINLIB_EXPORT_CLASS( ronex::AnalogueToJointPosition, pr2_mechanism_model::Transmission)

namespace ronex
{
  bool AnalogueToJointPosition::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    //read ronex name from urdf
    TiXmlElement *ronex_el = elt->FirstChildElement("ronex");
    const char *ronex_name = ronex_el ? ronex_el->Attribute("name") : NULL;
    if (!ronex_name)
    {
      ROS_ERROR("AnalogueToJointPosition transmission did not specify the ronex name");
      return false;
    }

    general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );
    if(!general_io_)
    {
      ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
    }

    //read ronex pin from urdf
    const char *ronex_pin = ronex_el ? ronex_el->Attribute("pin") : NULL;
    if (!ronex_pin)
    {
      ROS_ERROR("AnalogueToJointPosition transmission did not specify the ronex pin.");
      return false;
    }
    //convert pin to size_t and check it's in the correct bounds
    try
    {
      pin_index_ = boost::lexical_cast<size_t>( ronex_pin );
    }
    catch( boost::bad_lexical_cast const& )
    {
      ROS_ERROR("AnalogueToJointPosition: Couldn't parse pin to an int.");
    }
    if( pin_index_ >= general_io_->state_.analogue_.size() )
    {
      //size_t is always >= 0 so no need to check lower bound
      ROS_ERROR_STREAM("Specified pin is out of bound: " << pin_index_ << " / max = " << general_io_->state_.analogue_.size() << ".");
    }

    //read joint name
    TiXmlElement *jel = elt->FirstChildElement("joint");
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("AnalogueToJointPosition did not specify joint name");
      return false;
    }

    const boost::shared_ptr<const urdf::Joint> joint = robot->robot_model_.getJoint(joint_name);
    if (!joint)
    {
      ROS_ERROR("AnalogueToJointPosition could not find joint named \"%s\"", joint_name);
      return false;
    }
    joint_names_.push_back(joint_name);

    return true;
  }

  bool AnalogueToJointPosition::initXml(TiXmlElement *elt)
  {
    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    //read ronex name from urdf
    TiXmlElement *ronex_el = elt->FirstChildElement("ronex");
    const char *ronex_name = ronex_el ? ronex_el->Attribute("name") : NULL;
    if (!ronex_name)
    {
      ROS_ERROR("AnalogueToJointPosition transmission did not specify the ronex name");
      return false;
    }

    //read ronex pin from urdf
    const char *ronex_pin = ronex_el ? ronex_el->Attribute("pin") : NULL;
    if (!ronex_pin)
    {
      ROS_ERROR("AnalogueToJointPosition transmission did not specify the ronex pin.");
      return false;
    }
    //convert pin to size_t and check it's in the correct bounds
    try
    {
      pin_index_ = boost::lexical_cast<size_t>( ronex_pin );
    }
    catch( boost::bad_lexical_cast const& )
    {
      ROS_ERROR("AnalogueToJointPosition: Couldn't parse pin to an int.");
    }
    if( pin_index_ >= general_io_->state_.analogue_.size() )
    {
      //size_t is always >= 0 so no need to check lower bound
      ROS_ERROR_STREAM("Specified pin is out of bound: " << pin_index_ << " / max = " << general_io_->state_.analogue_.size() << ".");
    }

    //read joint name
    TiXmlElement *jel = elt->FirstChildElement("joint");
    const char *joint_name = jel ? jel->Attribute("name") : NULL;
    if (!joint_name)
    {
      ROS_ERROR("AnalogueToJointPosition did not specify joint name");
      return false;
    }
    joint_names_.push_back(joint_name);

    return true;
  }

  void AnalogueToJointPosition::propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                                                  std::vector<pr2_mechanism_model::JointState*>& js)
  {
    assert(js.size() == 1);

    //@todo calibrate here?
    js[0]->position_ = general_io_->state_.analogue_[pin_index_];
  }

  void AnalogueToJointPosition::propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>& js,
                                                           std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    //not doing anything: this transmission only maps an analogue pin to the position of a given joint
  }

  void AnalogueToJointPosition::propagateEffort(std::vector<pr2_mechanism_model::JointState*>& js,
                                                std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    //not doing anything: this transmission only maps an analogue pin to the position of a given joint
  }

  void AnalogueToJointPosition::propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>& js,
                                                         std::vector<pr2_mechanism_model::JointState*>& as)
  {
    //not doing anything: this transmission only maps an analogue pin to the position of a given joint
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
