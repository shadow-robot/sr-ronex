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

#ifndef SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_H_
#define SR_RONEX_GENERAL_IO_COMMAND_TO_PWM_H_

#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include "sr_ronex_mechanism_model/mapping/ronex_mapping.hpp"

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
        CommandToPWM() {};
        CommandToPWM(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot);
        virtual ~CommandToPWM();

        /**
         * This function is not doing anything as we're not propagating a status in this mapping.
         */
        virtual void propagateFromRonex(std::vector<pr2_mechanism_model::JointState*>& js) {};

        /**
         * Propagating the specified joint command to the given PWM module.
         *
         * @param js joint_state of the joint specified in the transmission
         */
        virtual void propagateToRonex(std::vector<pr2_mechanism_model::JointState*>& js);

      protected:
        ///Pointer to the GeneralIO module we specified in the transmission.
        GeneralIO* general_io_;
        ///PWM module index and PWM pin (0 or 1) as we have 2 pins per pwm_module_
        size_t pwm_module_, pin_index_;
        ///Are the pwm_module_ and pin_index_ in the correct ranges?
        bool pin_out_of_bound_;

        ///Those are used for computing the PWM on time / PWM period
        unsigned long int ideal_period_;
        unsigned short int clock_divider_, actual_period_, on_time_;

        /**
         * Check whether the pwm_module_ and pin_index_ are in the correct ranges.
         * @return true if the pins are in the correct ranges
         */
        bool check_pins_in_bound_();
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
