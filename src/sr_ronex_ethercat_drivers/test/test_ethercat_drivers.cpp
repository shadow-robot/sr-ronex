/**
* @file test_utils.cpp
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
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program. If not, see <http://www.gnu.org/licenses/>.
*
*
* @brief Testing the Utility library in ronex_utils.hpp
*
*
*/

#include <stdint.h>
#include <sr_ronex_external_protocol/Ronex_Protocol_0x02000001_GIO_00.h>
#include <sr_ronex_ethercat_drivers/ronex_utils.hpp>
#include <al/ethercat_slave_handler.h>
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
  ASSERT_TRUE( check_bit(data, 1 ) );

  set_bit(data, 4, false);
  ASSERT_FALSE( check_bit(data, 4 ) );
}

TEST(RonexUtils, build_name )
{
  const EC_UDINT serial = 55662211;
  EtherCAT_FMMU_Config *fmmu = new EtherCAT_FMMU_Config(0);
  EtherCAT_PD_Config *pdcfg = new EtherCAT_PD_Config(0);
  EtherCAT_SlaveHandler *sh = new EtherCAT_SlaveHandler( 0, 0, 0, serial, EC_FixedStationAddress( (EC_UINT) 0 ), fmmu, pdcfg, 0 );

  ostringstream ostr;
  ostr << "ronex_0_" << serial;
  string result = build_name(sh);

  ASSERT_STREQ( result.c_str(), ostr.str().c_str() );

  delete fmmu;
  delete pdcfg;
  delete sh;
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
