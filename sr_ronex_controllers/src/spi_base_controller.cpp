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
 * @file   spi_base_controller.hpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  A base controller for the SPI module.
 **/

#include "sr_ronex_controllers/spi_base_controller.hpp"
#include "pluginlib/class_list_macros.h"

namespace ronex
{
  SPIBaseController::SPIBaseController()
    : loop_count_(0)
  {
    command_queue_.resize(NUM_SPI_OUTPUTS);
  }

  SPIBaseController::~SPIBaseController()
  {}

  bool SPIBaseController::init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n)
  {
    assert(robot);
    node_ = n;

    std::string ronex_id;
    if (!node_.getParam("ronex_id", ronex_id)) {
      ROS_ERROR("No RoNeX ID given (namespace: %s)", node_.getNamespace().c_str());
      return false;
    }

    //get the path from the parameters
    std::string path;
    int parameter_id = get_ronex_param_id(ronex_id);
    {
      if( parameter_id == -1 )
      {
        ROS_ERROR_STREAM("Could not find the RoNeX id in the parameter server: " << ronex_id << " not loading the controller.");
        return false;
      }
      else
      {
        std::stringstream ss;
        ss << "/ronex/devices/" << parameter_id << "/path";
        if( !ros::param::get(ss.str(), path) )
        {
          ROS_ERROR_STREAM("Couldn't read the parameter " << ss.str() << " from the parameter server. Not loading the controller.");
          return false;
        }
      }
    }

    topic_prefix_ = path;

    spi_ = static_cast<ronex::SPI*>( robot->model_->hw_->getCustomHW(path) );
    if( spi_ == NULL)
    {
      ROS_ERROR_STREAM("Could not find RoNeX module: " << ronex_id << " not loading the controller");
      return false;
    }

    return true;
  }

  void SPIBaseController::starting()
  {
    ROS_ERROR("@TODO: implement starting");
  }

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void SPIBaseController::update()
  {
    ROS_ERROR("@TODO: implement update");

    //if no available command then send the NULL command
    for (size_t spi_index = 0; spi_index < NUM_SPI_OUTPUTS; ++spi_index)
    {
      if( command_queue_[spi_index].empty() )
      {
        spi_->nullify_command_(spi_index);
      }
    }
  }
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/
