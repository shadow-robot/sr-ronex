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
* @file command_to_pwm.hpp
* @author Ugo Cupcic <ugo@shadowrobot.com>
* @brief Contains the data mapping one joint command to a pwm module.
**/

#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm.hpp"
#include <ros_ethercat_model/robot_state.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      CommandToPWM::CommandToPWM(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot)
        : RonexMapping(), pin_out_of_bound_(true)
      {
        const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
        if (!ronex_name)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
          return;
        }

        init_timer_ = nh_.createTimer(ros::Duration(0.01),
                                      boost::bind(&CommandToPWM::try_init_cb_, this, _1, mapping_el, robot, ronex_name));
      }

      bool CommandToPWM::try_init_cb_(const ros::TimerEvent&, TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name)
      {
        //has the ronex been added by the driver?
        if( robot->getCustomHW(ronex_name) == NULL )
          return false;

        general_io_ = static_cast<ronex::GeneralIO*>( robot->getCustomHW(ronex_name) );
        if(!general_io_)
        {
          ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
          return false;
        }

        //read PWM module index from urdf
        const char *pwm_module = mapping_el ? mapping_el->Attribute("pwm_module") : NULL;
        if (!pwm_module)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm module.");
          return false;
        }
        //convert to size_t
        try
        {
          pwm_module_ = boost::lexical_cast<size_t>( pwm_module );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_module to a size_t.");
          return false;
        }

        //read PWM pin index from urdf
        const char *pin = mapping_el ? mapping_el->Attribute("pwm_pin") : NULL;
        if (!pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm pin.");
          return false;
        }
        //convert to size_t
        try
        {
          pwm_pin_index_ = boost::lexical_cast<size_t>( pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_pin to a size_t.");
          return false;
        }

        //read motor direction pin index from urdf
        const char *d_pin = mapping_el ? mapping_el->Attribute("direction_pin") : NULL;
        if (!d_pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the direction pin.");
          return false;
        }
        //convert to size_t
        try
        {
          digital_pin_index_ = boost::lexical_cast<size_t>( d_pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse direction_pin to a size_t.");
          return false;
        }

        ROS_DEBUG_STREAM("RoNeX" << ronex_name << " is initialised now.");
        //stopping timer
        init_timer_.stop();

        is_initialized_ = true;
        return true;
      }

      bool CommandToPWM::check_pins_in_bound_()
      {
        //we ignore the first iteration as the array is not yet initialised.
        if( first_iteration_ )
        {
          pin_out_of_bound_ = true;
          first_iteration_ = false;
          return false;
        }

        //we have to check here for the size otherwise the general io hasn't been updated.
        if( pin_out_of_bound_ )
        {
          if( pwm_module_ >= general_io_->command_.pwm_.size() )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified PWM module index is out of bound: " << pwm_pin_index_ << " / max = " << general_io_->command_.pwm_.size() << ", not propagating the command to the RoNeX.");
            pin_out_of_bound_ = true;
            return false;
          }
          if( pwm_pin_index_ > 1 )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified PWM pin is out of bound: " << pwm_pin_index_ << " / max = 1, not propagating the command to the RoNeX.");
            pin_out_of_bound_ = true;
            return false;
          }
          if( digital_pin_index_ > general_io_->command_.digital_.size() )
          {
            //size_t is always >= 0 so no need to check lower bound
            ROS_ERROR_STREAM("Specified direction pin is out of bound: " << digital_pin_index_ << " / max = " << general_io_->command_.digital_.size() << " , not propagating the command to the RoNeX.");
            pin_out_of_bound_ = true;
            return false;
          }
        }

        pin_out_of_bound_ = false;
        return true;
      }

      void CommandToPWM::propagateToRonex(ros_ethercat_model::JointState *js)
      {
        if( !is_initialized_ )
          return;

        if( check_pins_in_bound_() )
        {
          if( pwm_pin_index_ == 0 )
            general_io_->command_.pwm_[pwm_module_].on_time_0 = static_cast<unsigned short int>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) * js->commanded_effort_ ) / 100);
          else
            general_io_->command_.pwm_[pwm_module_].on_time_1 = static_cast<unsigned short int>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) * js->commanded_effort_ ) / 100);

          // This is assigned by convention: negative effort sets the direction pin to 1.
          general_io_->command_.digital_[digital_pin_index_] = (js->commanded_effort_ < 0.0);
       }
      }
    }
  }
}
/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
