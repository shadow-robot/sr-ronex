/**
 * @file   general_io_passthrough_controller.hpp
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
 * @brief A passthrough for the General IO RoNeX module: simply sets the
 *        different pins to the value you want directly.
 *
 *
 */

#ifndef _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_
#define _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_

#include <ros/node_handle.h>

#include <pr2_controller_interface/controller.h>
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>

#include <std_msgs/Bool.h>
#include <sr_ronex_msgs/PWM.h>

namespace ronex
{
  /**
   * @todo: this is copied from ronex_utils.hpp - move to separate sr_ronex_utils package?
   * Checks the ronexes already present on the parameter server and returns an id on which
   *  the given ronex is stored on the parameter server.
   *
   * The parameter server contains:
   *  /ronex/0/product_id = "0x200001"
   *  /ronex/0/produc_name = "general_io"
   *  /ronex/0/ronex_id = "my_beautiful_ronex" or serial if no alias
   *  /ronex/0/path = "ronex/general_io/my_beautiful_ronex/"
   *  /ronex/0/serial = "1234"
   *
   *  /ronex/1/...
   *
   * @param ronex_id Either the alias or the serial number if no alias is specified.
   *                 If empty string given, then returns the first available id.
   * @return the index of the ronex in the parameter server. -1 if not found.
   *         or the next free index if ronex_id == ""
   */
  static inline int get_ronex_param_id(std::string ronex_id)
  {
    std::string param;

    bool last_ronex = false;
    int ronex_parameter_id = 0;
    while( !last_ronex )
    {
      std::stringstream ss;
      ss << "/ronex/" << ronex_parameter_id << "/ronex_id";

      if(ros::param::get(ss.str(), param) )
      {
        if( ronex_id.compare("") != 0 )
        {
          if( ronex_id.compare(param) == 0)
          {
            return ronex_parameter_id;
          }
        }
        ++ronex_parameter_id;
      }
      else
      {
        if( ronex_id.compare("") != 0)
        {
          //we were looking for a specific ronex and didn't find it -> return -1
          return -1;
        }

        return ronex_parameter_id;
      }
    }

    return -1;
  }

  class GeneralIOPassthroughController
    : public pr2_controller_interface::Controller
  {
  public:
    GeneralIOPassthroughController();
    virtual ~GeneralIOPassthroughController();

    virtual bool init(pr2_mechanism_model::RobotState* robot, ros::NodeHandle &n);

    virtual void starting();

    /*!
     * \brief Issues commands to the joint. Should be called at regular intervals
     */
    virtual void update();

    void digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index);

    void pwm_commands_cb(const sr_ronex_msgs::PWMConstPtr& msg, int index);

  private:
    ros::NodeHandle node_;

    int loop_count_;

    ronex::GeneralIO* general_io_;

    ///send commands to the RoNeX's digital I/O
    std::vector<ros::Subscriber> digital_subscribers_;
    ///send PWM commands to the RoNeX's
    std::vector<ros::Subscriber> pwm_subscribers_;
  };
}

/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/

#endif /* _GENERAL_IO_PASSTHROUGH_CONTROLLER_H_ */
