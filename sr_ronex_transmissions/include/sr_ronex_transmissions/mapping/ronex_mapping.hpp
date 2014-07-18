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
 * @file   ronex_mapping.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Base class, contains the data mapping one element of a RoNeX to
 *         a joint element.
 **/

#ifndef _RONEX_MAPPING_H_
#define _RONEX_MAPPING_H_

#include <ros_ethercat_model/robot_state.hpp>
#include <ros_ethercat_model/transmission.hpp>

namespace ronex
{
  class RonexMapping
  {
  public:
    RonexMapping()
      : first_iteration_(true), is_initialized_(false)
    {};

    /**
     * Propagating the data from the RoNeXes to the joint states. This function is
     *  implemented in the different mappings.
     *
     * @param js joint_state of the joint specified in the transmission
     */
    virtual void propagateFromRonex(ros_ethercat_model::JointState *js) = 0;

    /**
     * Propagating the commands from joint states to the RoNeXes. This function is
     *  implemented in the different mappings.
     *
     * @param js joint_state of the joint specified in the transmission
     */
    virtual void propagateToRonex(ros_ethercat_model::JointState *js) = 0;

  protected:
    ros::NodeHandle nh_;
    bool first_iteration_;

    /// this is false until the RoNeX driver has initialised the RoNeX we're waiting for
    bool is_initialized_;
    /// using a timer to initialize the transmission once we find the RoNeX we're looking for.
    ros::Timer init_timer_;
    /**
     * Timer callback for the transmission initialization. Stops the init_timer_ when the
     *  initialization is successful.
     */
    virtual bool try_init_cb_(const ros::TimerEvent&, TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name) = 0;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_MAPPING_H_ */
