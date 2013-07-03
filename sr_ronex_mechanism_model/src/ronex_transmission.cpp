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
 *         RoNeX module to the position of the joint.
 *
 *
 */

#include <pr2_mechanism_model/robot.h>
#include <sr_ronex_mechanism_model/ronex_transmission.hpp>
#include "pluginlib/class_list_macros.h"
#include <boost/lexical_cast.hpp>

PLUGINLIB_EXPORT_CLASS( ronex::RonexTransmission, pr2_mechanism_model::Transmission)

namespace ronex
{
  bool RonexTransmission::initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot)
  {
    pin_out_of_bound_ = true;

    const char *name = elt->Attribute("name");
    name_ = name ? name : "";

    //Extract all the mapping information from the transmission
    for( TiXmlElement *mapping_el = elt->FirstChildElement("mapping"); mapping_el;
         mapping_el = mapping_el->NextSiblingElement("mapping") )
    {
//      ronex_mappings_.push_back( new RonexMapping(mapping_el, robot) );
    }

    return true;
  }

  bool RonexTransmission::initXml(TiXmlElement *elt)
  {
    pin_out_of_bound_ = true;

    return true;
  }

  void RonexTransmission::propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                                                  std::vector<pr2_mechanism_model::JointState*>& js)
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

  void RonexTransmission::propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>& js,
                                                           std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    //not doing anything: this transmission only maps an analogue pin to the position of a given joint
  }

  void RonexTransmission::propagateEffort(std::vector<pr2_mechanism_model::JointState*>& js,
                                                std::vector<pr2_hardware_interface::Actuator*>& as)
  {
    //not doing anything: this transmission only maps an analogue pin to the position of a given joint
  }

  void RonexTransmission::propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>& js,
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
