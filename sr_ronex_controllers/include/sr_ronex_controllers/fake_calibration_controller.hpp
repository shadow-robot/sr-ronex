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
 * @file   fake_calibration_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Those controllers fake the calibration, setting the joints as
 *         "calibrated" - otherwise the joints can't be controlled.
 **/

#ifndef _FAKE_CALIBRATION_CONTROLLER_H_
#define _FAKE_CALIBRATION_CONTROLLER_H_

#include <ros/node_handle.h>

#include <boost/smart_ptr.hpp>
#include "ros_ethercat_model/robot_state.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Bool.h"
#include <controller_interface/controller.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>

namespace ronex
{
  class FakeCalibrationController
    : public controller_interface::Controller<ros_ethercat_model::RobotState>
  {
  public:
    FakeCalibrationController();

    virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update(const ros::Time&, const ros::Duration&);

  private:
    ros_ethercat_model::RobotState* robot_;
    ros::NodeHandle node_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool> > pub_calibrated_;
    ros::Time last_publish_time_;

    enum { INITIALIZED, BEGINNING, MOVING_TO_LOW, MOVING_TO_HIGH, CALIBRATED };
    int state_;

    ros_ethercat_model::JointState *joint_;
    std::string joint_name_;

    std_msgs::Bool calib_msg_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _FAKE_CALIBRATION_CONTROLLER_H_ */
