/*
 * Copyright (c) 2015, Shadow Robot Company, All rights reserved.
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
 * @file   command_to_pwm_2_dir_pin.hpp
 * @author Toni Oliver <toni@shadowrobot.com>
 * @brief  Contains the data mapping one joint command to a pwm module. It uses 2 opposite direction pins (unlike CommandToPWM, that uses a single one)
 **/

#ifndef SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_2_DIR_PIN_H_
#define SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_2_DIR_PIN_H_

#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include "sr_ronex_transmissions/mapping/general_io/command_to_pwm.hpp"

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      class CommandToPWM2PinDir
        : public CommandToPWM
      {
      public:
    	CommandToPWM2PinDir(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot);
    	virtual ~CommandToPWM2PinDir(){};

    	/**
         * This function is not doing anything as we're not propagating a status in this mapping.
         */
        virtual void propagateFromRonex(ros_ethercat_model::JointState *js) {};

        /**
         * Propagating the specified joint command to the given PWM module.
         *
         * @param js joint_state of the joint specified in the transmission
         */
        virtual void propagateToRonex(ros_ethercat_model::JointState *js);

      protected:
        ///digital pin index for the second motor direction digital pin
        size_t digital_pin_index_2_;

        /**
         * Check whether the pwm_module_ and pin_index_ are in the correct ranges.
         * @return true if the pins are in the correct ranges
         */
        bool check_pins_in_bound_();

        virtual bool try_init_cb_(const ros::TimerEvent&, TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name);
        bool init_(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot, const char* ronex_name);
      };
    }
  }
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif  /* SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_2_DIR_PIN_H_ */
