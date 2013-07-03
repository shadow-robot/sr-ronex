/**
 * @file   ronex_transmission.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
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
 * @brief  A transmission for mapping RoNeXes to joints.
 *
 *
 */

#ifndef _RONEX_TRANSMISSION_H_
#define _RONEX_TRANSMISSION_H_

#include <pr2_mechanism_model/transmission.h>
#include <pr2_mechanism_model/joint.h>
#include <pr2_mechanism_model/joint_calibration_simulator.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
//#include "sr_ronex_mechanism_model/ronex_mapping.hpp"

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
    ronex::GeneralIO* general_io_;
    size_t pin_index_;
    bool pin_out_of_bound_;

    // boost::ptr_vector<RonexMapping> ronex_mappings_;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_TRANSMISSION_H_ */
