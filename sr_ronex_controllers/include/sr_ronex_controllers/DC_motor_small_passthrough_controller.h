/*
 * DC_motor_small_controller.h
 *
 *  Created on: 29 Dec 2015
 *      Author: hand
 */

#ifndef SR_RONEX_SR_RONEX_CONTROLLERS_INCLUDE_DC_MOTOR_SMALL_CONTROLLER_H_
#define SR_RONEX_SR_RONEX_CONTROLLERS_INCLUDE_DC_MOTOR_SMALL_CONTROLLER_H_

#include <ros/node_handle.h>

#include <controller_interface/controller.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <sr_ronex_hardware_interface/DC_motor_small_hardware_interface.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <sr_ronex_utilities/sr_ronex_utilities.hpp>

#include <std_msgs/Bool.h>
#include "sr_ronex_msgs/MotorPacketStatus.h"
#include <vector>

namespace ronex
{
class DCMotorSmallPassthroughController
    : public controller_interface::Controller<ros_ethercat_model::RobotState>
{
public:
  DCMotorSmallPassthroughController();

  virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  virtual void update(const ros::Time&, const ros::Duration&) {}

  void digital_commands_cb(const std_msgs::BoolConstPtr& msg, int index);

  void motor_packet_cb(const sr_ronex_msgs::MotorPacketStatusConstPtr &msg, int index);

private:
  ros::NodeHandle node_;

  int loop_count_;

  ronex::DCMotor* dc_motor_small_;

  /// send commands to the RoNeX's digital I/O
  std::vector<ros::Subscriber> digital_subscribers_;

  /// send commands to the RoNeX's Motor controllers
  std::vector<ros::Subscriber> motor_command_subscribers_;

};
}  // namespace ronex
#endif /* SR_RONEX_SR_RONEX_CONTROLLERS_INCLUDE_DC_MOTOR_SMALL_CONTROLLER_H_ */
