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
#include <sr_ronex_mechanism_model/analogue_to_joint_effort.hpp>
#include "pluginlib/class_list_macros.h"
#include <boost/lexical_cast.hpp>

PLUGINLIB_EXPORT_CLASS( ronex::AnalogueToJointEffort, pr2_mechanism_model::Transmission)

namespace ronex
{
  void AnalogueToJointEffort::propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                                                std::vector<pr2_mechanism_model::JointState*>& js)
  {
    //Propagating the measured effort here - misleading function name.

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
    js[0]->measured_effort_ = general_io_->state_.analogue_[pin_index_];
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/
