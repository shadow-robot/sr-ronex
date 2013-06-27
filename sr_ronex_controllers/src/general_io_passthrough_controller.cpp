/**
 * @file   general_io_passthrough_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @date   Thu Jun 27 11:04:12 2013
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
*
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
      pwm_subscribers_.push_back(node_.subscribe<sr_common_msgs::PWM>(sub_topic.str(), 1, boost::bind(&GeneralIOPassthroughController::pwm_commands_cb, this, _1, i)));
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
    if(loop_count_ % 10 == 0)
    {
      ROS_ERROR_STREAM("pin 1" << general_io_->state_.digital_[1]);

      loop_count_ = 0;
    }
    loop_count_++;
  }

  void GeneralIOPassthroughController::digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index)
  {
    general_io_->command_.digital_[index] = msg->data;
  }

  void GeneralIOPassthroughController::pwm_commands_cb(const sr_common_msgs::PWMConstPtr& msg, int index)
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