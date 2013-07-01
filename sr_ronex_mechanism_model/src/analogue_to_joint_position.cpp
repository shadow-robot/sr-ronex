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

namespace ronex
{
  bool AnalogueToJointPosition::initXml(TiXmlElement *config, pr2_mechanism_model::Robot *robot)
  {
    std::string ronex_name;

    //@todo read ronex name / pin from urdf
    general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );

    return true;
  }

  bool AnalogueToJointPosition::initXml(TiXmlElement *config)
  {

    return true;
  }

  void AnalogueToJointPosition::propagatePosition(std::vector<pr2_hardware_interface::Actuator*>&,
                                                  std::vector<pr2_mechanism_model::JointState*>&)
  {

  }

  void AnalogueToJointPosition::propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>&,
                                                           std::vector<pr2_hardware_interface::Actuator*>&)
  {

  }

  void AnalogueToJointPosition::propagateEffort(std::vector<pr2_mechanism_model::JointState*>&,
                                                std::vector<pr2_hardware_interface::Actuator*>&)
  {

  }

  void AnalogueToJointPosition::propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>&,
                                                         std::vector<pr2_mechanism_model::JointState*>&)
  {

  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
