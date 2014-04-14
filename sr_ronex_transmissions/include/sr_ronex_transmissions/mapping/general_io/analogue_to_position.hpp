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
 * @file   analogue_to_position.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Contains the data mapping one pin of a GIO module to the position
 *         of the joint.
 **/

#ifndef SR_RONEX_GENERAL_IO_POSITION_MAPPING_H_
#define SR_RONEX_GENERAL_IO_POSITION_MAPPING_H_

#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include "sr_ronex_transmissions/mapping/ronex_mapping.hpp"

namespace ronex
{
  namespace mapping
  {
    namespace general_io
    {
      class AnalogueToPosition
        : public RonexMapping
      {
      public:
        AnalogueToPosition()
         : RonexMapping() {};
        AnalogueToPosition(TiXmlElement* mapping_el, ros_ethercat_model::RobotState* robot);
        virtual ~AnalogueToPosition();

        /**
         * Propagating the specified analogue pin data to the given joint position.
         *
         * @param js joint_state of the joint specified in the transmission
         */
        virtual void propagateFromRonex(std::vector<ros_ethercat_model::JointState*>& js);

        /**
         * This function is not doing anything as we're not propagating a command in this mapping.
         */
        virtual void propagateToRonex(std::vector<ros_ethercat_model::JointState*>& js) {};

      protected:
        ///Pointer to the GeneralIO module we specified in the transmission.
        GeneralIO* general_io_;
        ///index of the analogue pin
        size_t pin_index_;
        ///Is the pin inside the correct range?
        bool pin_out_of_bound_;

        ///The user can apply a scaling and offset to the raw data.
        double scale_, offset_;

        /**
         * Computes the scaled data from the raw value.
         * @return scaled data
         */
        double compute_scaled_data_();

        /**
         * Check whether the pin is in the correct range.
         * @return true if the pin is in the correct range
         */
        bool check_pin_in_bound_();
      };
    }
  }
}
/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* SR_RONEX_GENERAL_IO_POSITION_MAPPING_H_ */
