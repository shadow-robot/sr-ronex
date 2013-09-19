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
 * @file   ronex_transmission.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A transmission for mapping RoNeXes to joints.
 **/

#ifndef _RONEX_TRANSMISSION_H_
#define _RONEX_TRANSMISSION_H_

#include <pr2_mechanism_model/transmission.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/joint_calibration_simulator.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include "sr_ronex_transmissions/mapping/ronex_mapping.hpp"

namespace ronex
{
  class RonexTransmission : public pr2_mechanism_model::Transmission
  {
  public:
    RonexTransmission() {};
    virtual ~RonexTransmission() {};

    bool initXml(TiXmlElement *elt, pr2_mechanism_model::Robot *robot);
    bool initXml(TiXmlElement *elt);

    void propagatePosition(std::vector<pr2_hardware_interface::Actuator*>& as,
                           std::vector<pr2_mechanism_model::JointState*>& js);
    void propagatePositionBackwards(std::vector<pr2_mechanism_model::JointState*>& js,
                                    std::vector<pr2_hardware_interface::Actuator*>& as);
    void propagateEffort(std::vector<pr2_mechanism_model::JointState*>& js,
                         std::vector<pr2_hardware_interface::Actuator*>& as);
    void propagateEffortBackwards(std::vector<pr2_hardware_interface::Actuator*>& as,
                                  std::vector<pr2_mechanism_model::JointState*>& js);

  protected:
    boost::ptr_vector<RonexMapping> ronex_mappings_;
    boost::ptr_vector<RonexMapping>::iterator ronex_iter_;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_TRANSMISSION_H_ */
