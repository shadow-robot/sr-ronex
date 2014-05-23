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

#include "sr_ronex_controllers/fake_calibration_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::FakeCalibrationController, controller_interface::ControllerBase)

namespace ronex
{
  FakeCalibrationController::FakeCalibrationController()
    : robot_(NULL), last_publish_time_(0), state_(INITIALIZED),
      joint_(NULL)
  {
  }

  bool FakeCalibrationController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
  {
    robot_ = robot;
    node_ = n;
    // Joint

    std::string joint_name;
    if (!node_.getParam("joint", joint_name))
    {
      ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }
    if (!(joint_ = robot->getJointState(joint_name)))
    {
      ROS_ERROR("Could not find joint %s (namespace: %s)",
                joint_name.c_str(), node_.getNamespace().c_str());
      return false;
    }
    joint_name_ = joint_name;

    // "Calibrated" topic
    pub_calibrated_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::Bool>(node_, "/calibrated", 1));

    return true;
  }

  /*!
   * \brief Sets the joint to calibrated = true; Also publishes true to the calibrated topic
   */
  void FakeCalibrationController::update(const ros::Time&, const ros::Duration&)
  {
    assert(joint_);

    switch(state_)
    {
    case INITIALIZED:
      state_ = BEGINNING;
      break;
    case BEGINNING:
      joint_->calibrated_ = true;
      calib_msg_.data = true;
      state_ = CALIBRATED;
      //We add the following line to delay for some time the first publish and allow the correct initialization of the subscribers in calibrate.py
      last_publish_time_ = robot_->getTime();
      break;
    case CALIBRATED:
      if (pub_calibrated_)
      {
        if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
        {
          assert(pub_calibrated_);
          if (pub_calibrated_->trylock())
          {
            last_publish_time_ = robot_->getTime();
            pub_calibrated_->msg_ = calib_msg_;
            pub_calibrated_->unlockAndPublish();
          }
        }
      }
      break;
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
