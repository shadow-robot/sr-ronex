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
 * @file   command_to_pwm.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Contains the data mapping one joint command to a pwm module.
 **/

#ifndef SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_H_
#define SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_H_

#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include "sr_ronex_transmissions/mapping/ronex_mapping.hpp"

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      class CommandToPWM
        : public RonexMapping
      {
      public:
        CommandToPWM()
          : RonexMapping()
        {};

        CommandToPWM(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot);
        virtual ~CommandToPWM();

        /**
         * This function is not doing anything as we're not propagating a status in this mapping.
         */
        virtual void propagateFromRonex(std::vector<ros_ethercat_model::JointState*>& js) {};

        /**
         * Propagating the specified joint command to the given PWM module.
         *
         * @param js joint_state of the joint specified in the transmission
         */
        virtual void propagateToRonex(std::vector<ros_ethercat_model::JointState*>& js);

      protected:
        ///Pointer to the GeneralIO module we specified in the transmission.
        GeneralIO* general_io_;
        ///PWM module index and PWM pin (0 or 1) as we have 2 pins per pwm_module_
        size_t pwm_module_, pwm_pin_index_;
        ///digital pin index for the motor direction digital pin
        size_t digital_pin_index_;
        ///Are the pwm_module_, pwm_pin_index_ and digital_pin_index_ in the correct ranges?
        bool pin_out_of_bound_;

        ///Those are used for computing the PWM on time / PWM period
        unsigned long int ideal_period_;
        unsigned short int clock_divider_, actual_period_, on_time_;

        /**
         * Check whether the pwm_module_ and pin_index_ are in the correct ranges.
         * @return true if the pins are in the correct ranges
         */
        bool check_pins_in_bound_();

        virtual bool try_init_cb_(const ros::TimerEvent&, TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name);
      };
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif  /* SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_H_ */
