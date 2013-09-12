/**
 * @file   ronex_mapping.hpp
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
 * @brief  Base class, contains the data mapping one element of a RoNeX to a joint element.
 *
 *
 */

#ifndef _RONEX_MAPPING_H_
#define _RONEX_MAPPING_H_

#include <pr2_mechanism_model/transmission.h>

namespace ronex
{
  class RonexMapping
  {
  public:
    RonexMapping() {};
    RonexMapping(TiXmlElement* mapping_el) {};
    RonexMapping(TiXmlElement* mapping_el, pr2_mechanism_model::Robot* robot) {};
    virtual ~RonexMapping() {};

    /**
     * Propagating the data from the RoNeXes to the joint states. This function is
     *  implemented in the different mappings.
     *
     * @param js Current joint states.
     */
    virtual void propagateFromRonex(std::vector<pr2_mechanism_model::JointState*>& js) = 0;

    /**
     * Propagating the commands from joint states to the RoNeXes. This function is
     *  implemented in the different mappings.
     *
     * @param js Current joint states.
     */
    virtual void propagateToRonex(std::vector<pr2_mechanism_model::JointState*>& js) = 0;
  };
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

#endif /* _RONEX_MAPPING_H_ */
