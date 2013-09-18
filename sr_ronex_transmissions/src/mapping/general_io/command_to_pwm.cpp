/**
 * @file   command_to_pwm.hpp
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
*
 * @brief  Contains the data mapping one joint command to a pwm module.
 *
 *
 */

#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm.hpp"
#include <pr2_mechanism_model/robot.h>
#include <boost/lexical_cast.hpp>
#include <math.h>

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      CommandToPWM::CommandToPWM(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot)
        : pin_out_of_bound_(true)
      {
        const char *ronex_name = mapping_el ? mapping_el->Attribute("ronex") : NULL;
        if (!ronex_name)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the ronex name");
          return;
        }

        general_io_ = static_cast<ronex::GeneralIO*>( robot->hw_->getCustomHW(ronex_name) );
        if(!general_io_)
        {
          ROS_ERROR_STREAM("The RoNeX: " << ronex_name << " was not found on the system.");
          return;
        }

        //read PWM module index from urdf
        const char *pwm_module = mapping_el ? mapping_el->Attribute("pwm_module") : NULL;
        if (!pwm_module)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm module.");
          return;
        }
        //convert to size_t
        try
        {
          pwm_module_ = boost::lexical_cast<size_t>( pwm_module );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_module to a size_t.");
          return;
        }

        //read PWM pin index from urdf
        const char *pin = mapping_el ? mapping_el->Attribute("pwm_pin") : NULL;
        if (!pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the pwm pin.");
          return;
        }
        //convert to size_t
        try
        {
          pwm_pin_index_ = boost::lexical_cast<size_t>( pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse pwm_pin to a size_t.");
          return;
        }

        //read motor direction pin index from urdf
        const char *d_pin = mapping_el ? mapping_el->Attribute("direction_pin") : NULL;
        if (!d_pin)
        {
          ROS_ERROR("RonexTransmission transmission did not specify the direction pin.");
          return;
        }
        //convert to size_t
        try
        {
          digital_pin_index_ = boost::lexical_cast<size_t>( d_pin );
        }
        catch( boost::bad_lexical_cast const& )
        {
          ROS_ERROR("RonexTransmission: Couldn't parse direction_pin to a size_t.");
          return;
        }
      }

      CommandToPWM::~CommandToPWM()
      {
      }

      bool CommandToPWM::check_pins_in_bound_()
      {
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

      void CommandToPWM::propagateToRonex(std::vector<pr2_mechanism_model::JointState*>& js)
      {
        assert(js.size() == 1);

        if( check_pins_in_bound_() )
        {
          ROS_ERROR_STREAM("pin in bound");
          if( pwm_pin_index_ == 0 )
            general_io_->command_.pwm_[pwm_module_].on_time_0 = static_cast<unsigned short int>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) * js[0]->commanded_effort_ ) / 100);
          else
            general_io_->command_.pwm_[pwm_module_].on_time_1 = static_cast<unsigned short int>((static_cast<double>(general_io_->command_.pwm_[pwm_module_].period) * js[0]->commanded_effort_ ) / 100);

          // This is assigned by convention: negative effort sets the direction pin to 1.
          general_io_->command_.digital_[digital_pin_index_] = (js[0]->commanded_effort_ < 0.0);
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
