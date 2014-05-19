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
 * @file   test_utils.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>
 * @brief  Testing the Utility library in ronex_utils.hpp
 **/

#include <stdint.h>
#include "sr_ronex_external_protocol/Ronex_Protocol_0x02000001_GIO_00.h"
#include <ros_ethercat_hardware/ethercat_hardware.h>
#include "sr_ronex_drivers/ronex_utils.hpp"
#include "sr_ronex_drivers/sr_board_mk2_gio.hpp"
#include "sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp"
#include <ros_ethercat_model/robot_state.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <tinyxml.h>

using namespace std;
using namespace ronex;

TEST(RonexEthercatDrivers, build_name )
{
  string expected = "/ronex/general_io/beautiful_ronex";
  string result = build_name("general_io", "beautiful_ronex");
  int res = expected.compare(result);
  ASSERT_EQ(0, res);
}

TEST(RonexEthercatDrivers, constructor )
{
  const uint32_t serial = 55662211;

  EtherCAT_FMMU_Config fmmu(0);
  EtherCAT_PD_Config pdcfg(0);
  EtherCAT_SlaveHandler sh(0, 0, 0, serial,EC_FixedStationAddress( (uint16_t) 0 ), &fmmu, &pdcfg, 0);

  SrBoardMk2GIO sbm;

  int add = 0;
  sbm.construct( &sh, add );

  ros::NodeHandle nh;
  string xml_string;
  nh.getParam("robot_description", xml_string);
  TiXmlDocument urdf_xml;
  urdf_xml.Parse(xml_string.c_str());
  TiXmlElement *root = urdf_xml.FirstChildElement("robot");
  ros_ethercat_model::RobotState hw(root);

  int retsbm = sbm.initialize( static_cast<hardware_interface::HardwareInterface*>(&hw) );
  ASSERT_EQ(0, retsbm);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_ethercat_drivers_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
