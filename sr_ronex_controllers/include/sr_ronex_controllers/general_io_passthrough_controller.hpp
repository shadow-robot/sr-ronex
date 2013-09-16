/**
 * @file   general_io_passthrough_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 *
* Copyright 2013 Shadow Robot Company Ltd.
*
* This program is Proprietary software: you cannot redistribute it or modify it
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.
 *
 * @brief A passthrough for the General IO RoNeX module: simply sets the
 *        different pins to the value you want directly.
 *
 *
 */

#ifndef _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_
#define _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Bool.h>
#include <sr_ronex_msgs/PWM.h>

namespace ronex
{
  class GeneralIOPassthroughController
    : public pr2_controller_interface::Controller
  {
  public:
    GeneralIOPassthroughController();
    virtual ~GeneralIOPassthroughController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    void digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index);

    void pwm_commands_cb(const sr_ronex_msgs::PWMConstPtr& msg, int index);

  private:
    ros::NodeHandle node_;

    int loop_count_;

    ronex::GeneralIO* general_io_;

    ///send commands to the RoNeX's digital I/O
    std::vector<ros::Subscriber> digital_subscribers_;
    ///send PWM commands to the RoNeX's
    std::vector<ros::Subscriber> pwm_subscribers_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_ */
