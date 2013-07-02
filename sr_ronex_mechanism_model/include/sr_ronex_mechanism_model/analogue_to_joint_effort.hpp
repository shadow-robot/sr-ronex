/**
 * @file   analogue_to_joint_effort.hpp
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
 * @brief  A transmission for mapping one analogue input pin from a general I/O
 *         RoNeX module to the effort of the joint.
 *
 *
 */

#ifndef _ANALOGUE_TO_JOINT_EFFORT_H_
#define _ANALOGUE_TO_JOINT_EFFORT_H_

#include <sr_ronex_mechanism_model/analogue_to_joint_position.hpp>

namespace ronex
{
  class AnalogueToJointEffort : public ronex::AnalogueToJointPosition
  {
  public:
    AnalogueToJointEffort() {};
    virtual ~AnalogueToJointEffort() {};

    void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                           std::vector<pr2_mechanism_model::JointState*>& js);
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _ANALOGUE_TO_JOINT_EFFORT_H_ */
