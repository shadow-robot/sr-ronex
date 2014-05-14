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
 * @file   test_ronex_transmission.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Testing the ronex transmission for a given urdf.
 **/

#include "sr_ronex_transmissions/ronex_transmission.hpp"
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <ros/ros.h>
#include <ros_ethercat_model/robot_state.hpp>
#include <gtest/gtest.h>

using namespace ronex;

TEST(RonexTransmission, constructor)
{
  //get urdf from param
  ros::NodeHandle nh;
  std::string xml_string;
  ASSERT_TRUE( nh.getParam("robot_description", xml_string) );
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());
  ros_ethercat_model::RobotState state(0);
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);

  //add ronex
  std::string name("/ronex/general_io/0");
  state.custom_hws_.insert(name, new ronex::GeneralIO());
  state.initXml(root);

  ronex::GeneralIO* general_io = static_cast<ronex::GeneralIO*>( state.getCustomHW(name) );
  ASSERT_TRUE(general_io != NULL);
}

TEST(RonexTransmission, propagateCommand)
{
  //get urdf from param
  ros::NodeHandle nh;
  std::string xml_string;
  ASSERT_TRUE( nh.getParam("robot_description", xml_string) );
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());
  ros_ethercat_model::RobotState state(0);
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);

  //add ronex
  std::string name("/ronex/general_io/0");
  state.custom_hws_.insert(name, new ronex::GeneralIO());
  state.initXml(root);

  ronex::GeneralIO* general_io = static_cast<ronex::GeneralIO*>( state.getCustomHW(name) );
  ASSERT_TRUE(general_io != NULL);
  general_io->command_.pwm_clock_divider_ = 20;
  general_io->command_.pwm_.resize(6);
  general_io->state_.analogue_.resize(6);
  general_io->state_.digital_.resize(6);
  general_io->command_.pwm_.resize(6);
  general_io->command_.digital_.resize(6);

  //setting effort for the joint
  state.joint_states_["joint1"].commanded_effort_ = 5.1;

  general_io->command_.pwm_[1].period =  64000;  //setting period too

  state.propagateJointEffortToActuatorEffort();
  state.propagateJointEffortToActuatorEffort();

  //reading the command from the RoNeX
  EXPECT_EQ(general_io->command_.pwm_[1].on_time_0, 3264);
  EXPECT_FALSE(general_io->command_.digital_[5]);  //Positive commanded effort should set direction pin to 0
}


TEST(RonexTransmission, propagateState)
{
  //get urdf from param
  ros::NodeHandle nh;
  std::string xml_string;
  ASSERT_TRUE( nh.getParam("robot_description", xml_string) );
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());
  ros_ethercat_model::RobotState state(0);
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);

  //add ronex
  std::string name("/ronex/general_io/0");
  state.custom_hws_.insert(name, new ronex::GeneralIO());
  state.initXml(root);

  ronex::GeneralIO* general_io = static_cast<ronex::GeneralIO*>( state.getCustomHW(name) );
  ASSERT_TRUE(general_io != NULL);
  general_io->command_.pwm_.resize(6);
  general_io->state_.analogue_.resize(6);
  general_io->state_.digital_.resize(6);
  general_io->command_.pwm_.resize(6);

  //setting analogue data on the ronex for generating joint position / effort
  general_io->state_.analogue_[0] = 1.0; //position according to urdf
  general_io->state_.analogue_[1] = 1.0; //mapped to effort
  //propagating
  state.propagateActuatorPositionToJointPosition();
  state.propagateActuatorPositionToJointPosition();
  //reading the position and effort from the RoNeX
  EXPECT_DOUBLE_EQ(state.joint_states_["joint1"].position_, 1.0); //scale is 1.0, offset 0.0
  EXPECT_DOUBLE_EQ(state.joint_states_["joint1"].measured_effort_, 3.0); //scale is 2.0, offset 1.0
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_ronex_transmissions");
    ros::NodeHandle nh; // init the node
    return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
   Local Variables:
   c-basic-offset: 2
   End:
*/

