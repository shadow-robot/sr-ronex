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

#include "sr_ronex_controllers/general_io_passthrough_controller.hpp"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_EXPORT_CLASS( ronex::GeneralIOPassthroughController, pr2_controller_interface::Controller)

namespace ronex
{
  GeneralIOPassthroughController::GeneralIOPassthroughController()
    : loop_count_(0)
  {}

  GeneralIOPassthroughController::~GeneralIOPassthroughController()
  {
    for(size_t i=0; i < digital_subscribers_.size(); ++i)
    {
      digital_subscribers_[i].shutdown();
    }
    for(size_t i=0; i < pwm_subscribers_.size(); ++i)
    {
      pwm_subscribers_[i].shutdown();
    }
  }

  bool GeneralIOPassthroughController::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string ronex_name;
    if (!node_.getParam("ronex", ronex_name)) {
      ROS_ERROR("No RoNeX given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    general_io_ = static_cast<ronex::GeneralIO*>( robot->model_->hw_->getCustomHW(ronex_name) );
    if( general_io_ == NULL)
    {
      ROS_ERROR_STREAM("Could not find RoNeX module: " << ronex_name << " not loading the controller");
      return false;
    }

    //init the subscribers
    std::stringstream sub_topic;
    for( size_t i=0; i < general_io_->command_.digital_.size(); ++i)
    {
      sub_topic.str("");
      sub_topic << ronex_name << "/command/digital/" << i;
      digital_subscribers_.push_back(node_.subscribe<std_msgs::Bool>(sub_topic.str(), 1, boost::bind(&GeneralIOPassthroughController::digital_commands_cb, this, _1,  i )));
    }

    for( size_t i=0; i < general_io_->command_.pwm_.size(); ++i)
    {
      sub_topic.str("");
      sub_topic << ronex_name << "/command/pwm/" << i;
      pwm_subscribers_.push_back(node_.subscribe<sr_ronex_msgs::PWM>(sub_topic.str(), 1, boost::bind(&GeneralIOPassthroughController::pwm_commands_cb, this, _1, i)));
    }

    return true;
  }

  void GeneralIOPassthroughController::starting()
  {}

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void GeneralIOPassthroughController::update()
  {
/*
    if(loop_count_ % 10 == 0)
    {
      loop_count_ = 0;
    }
    loop_count_++;
*/
  }

  void GeneralIOPassthroughController::digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index)
  {
    general_io_->command_.digital_[index] = msg->data;
  }

  void GeneralIOPassthroughController::pwm_commands_cb(const sr_ronex_msgs::PWMConstPtr& msg, int index)
  {

    general_io_->command_.pwm_[index].period = msg->pwm_period;
    general_io_->command_.pwm_[index].on_time_0 = msg->pwm_on_time_0;
    general_io_->command_.pwm_[index].on_time_1 = msg->pwm_on_time_1;
  }

}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
