/**
* @file test_utils.cpp
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
* @brief Testing the Utility library in ronex_utils.hpp
*
*
*/

#include <stdint.h>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000001_GIO_00.h>
#include <al/ethercat_slave_handler.h>
#include "sr_ronex_ethercat_drivers/ronex_utils.hpp"
#include "sr_ronex_ethercat_drivers/sr_board_mk2_gio.hpp"
#include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
#include <ros/ros.h>
#include <gtest/gtest.h>

using namespace std;
using namespace ronex;

TEST(RonexUtils, set_bit)
{
  int32u data = 0;
  set_bit(data, 1, true);
  EXPECT_EQ(data, 2);

  set_bit(data, 1, false);
  EXPECT_EQ(data, 0);

  set_bit(data, 1, true);
  set_bit(data, 2, true);
  set_bit(data, 3, true);
  EXPECT_EQ(data, 14);

  for ( size_t i = 0; i < 32; ++i )
    set_bit(data, i, true);
  EXPECT_EQ(data, 4294967295);

  set_bit(data, 1, true);
  EXPECT_TRUE( check_bit(data, 1 ) );

  set_bit(data, 4, false);
  EXPECT_FALSE( check_bit(data, 4 ) );
}

TEST(RonexUtils, build_name )
{
  const EC_UDINT serial = 55662211;

  EtherCAT_FMMU_Config fmmu(0);
  EtherCAT_PD_Config pdcfg(0);
  EtherCAT_SlaveHandler sh(0, 0, 0, serial,EC_FixedStationAddress( (EC_UINT) 0 ), &fmmu, &pdcfg, 0);

  ostringstream ostr;
  ostr << "ronex_0_" << serial;
  string result = build_name(&sh);

  EXPECT_STREQ( result.c_str(), ostr.str().c_str() );
}

TEST(RonexUtils, constructor )
{
  const EC_UDINT serial = 55662211;

  EtherCAT_FMMU_Config fmmu(0);
  EtherCAT_PD_Config pdcfg(0);
  EtherCAT_SlaveHandler sh(0, 0, 0, serial,EC_FixedStationAddress( (EC_UINT) 0 ), &fmmu, &pdcfg, 0);

  SrBoardMk2GIO sbm;

  int add = 0;
  sbm.construct( &sh, add );

  pr2_hardware_interface::HardwareInterface hw;
  int retsbm = sbm.initialize( &hw );

  EXPECT_EQ(retsbm,0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init( std::map<std::string, std::string>(), "test_ethercat_drivers");
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/