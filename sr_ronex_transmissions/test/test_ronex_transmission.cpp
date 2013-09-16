/**
 * @file   test_ronex_transmission.cpp
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
 * @brief Testing the ronex transmission for a given urdf.
 *
 *
 */

#include "sr_ronex_transmissions/ronex_transmission.hpp"
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <ros/ros.h>
#include <pr2_mechanism_model/robot.h>
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

  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);
  pr2_hardware_interface::HardwareInterface hw;

  //add ronex
  boost::shared_ptr<ronex::GeneralIO> general_io;
  general_io.reset( new ronex::GeneralIO() );
  general_io->name_ = "ronex_33554433_0";
  hw.addCustomHW( general_io.get() );

  pr2_mechanism_model::Robot model(&hw);
  ASSERT_TRUE(model.initXml(root));
  pr2_mechanism_model::RobotState state(&model);
}

TEST(RonexTransmission, propagateCommand)
{
  //get urdf from param
  ros::NodeHandle nh;
  std::string xml_string;
  ASSERT_TRUE( nh.getParam("robot_description", xml_string) );

  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());

  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);
  pr2_hardware_interface::HardwareInterface hw;

  //add ronex
  boost::shared_ptr<ronex::GeneralIO> general_io;
  general_io.reset( new ronex::GeneralIO() );
  general_io->name_ = "ronex_33554433_0";
  general_io->command_.pwm_clock_divider_ = 20;
  general_io->command_.pwm_.resize(6);
  general_io->state_.analogue_.resize(6);
  general_io->state_.digital_.resize(6);
  general_io->command_.pwm_.resize(6);

  hw.addCustomHW( general_io.get() );

  pr2_mechanism_model::Robot model(&hw);
  ASSERT_TRUE(model.initXml(root));

  pr2_mechanism_model::RobotState state(&model);

  //setting effort for the joint
  state.joint_states_[0].commanded_effort_ = 5.1;

  general_io->command_.pwm_[1].period =  64000;  //setting period too

  state.propagateJointEffortToActuatorEffort();
  //reading the command from the RoNeX
  EXPECT_EQ(general_io->command_.pwm_[1].on_time_0, 3264);

}


TEST(RonexTransmission, propagateState)
{
  //get urdf from param
  ros::NodeHandle nh;
  std::string xml_string;
  ASSERT_TRUE( nh.getParam("robot_description", xml_string) );

  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());

  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ASSERT_TRUE(root != NULL);
  pr2_hardware_interface::HardwareInterface hw;

  //add ronex
  boost::shared_ptr<ronex::GeneralIO> general_io;
  general_io.reset( new ronex::GeneralIO() );
  general_io->name_ = "ronex_33554433_0";
  general_io->command_.pwm_.resize(6);
  general_io->state_.analogue_.resize(6);
  general_io->state_.digital_.resize(6);
  general_io->command_.pwm_.resize(6);

  hw.addCustomHW( general_io.get() );

  pr2_mechanism_model::Robot model(&hw);
  ASSERT_TRUE(model.initXml(root));
  pr2_mechanism_model::RobotState state(&model);

  //setting analogue data on the ronex for generating joint position / effort
  general_io->state_.analogue_[0] = 1.0; //position according to urdf
  general_io->state_.analogue_[1] = 1.0; //mapped to effort
  //propagating
  state.propagateActuatorPositionToJointPosition();
  //reading the position and effort from the RoNeX
  EXPECT_DOUBLE_EQ(state.joint_states_[0].position_, 1.0); //scale is 1.0, offset 0.0
  EXPECT_DOUBLE_EQ(state.joint_states_[0].measured_effort_, 3.0); //scale is 2.0, offset 1.0
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

