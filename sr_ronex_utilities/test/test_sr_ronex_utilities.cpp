/**
* @file test_utilities.cpp
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
* @brief Testing the Utility library in sr_ronex_utilities.hpp
*
*
*/

#include <stdint.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

#include "sr_ronex_utilities/sr_ronex_utilities.hpp"
using namespace std;
using namespace ronex;

TEST(RonexUtils, set_bit)
{
  uint32_t data = 0;
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

TEST(RonexUtils, get_ronex_param_id)
{
  //make sure we're running on a clean environment
  ros::param::del("/ronex");
  //nothing in the parameter server -> ronex param id = 0
  int ronex_param_id = get_ronex_param_id("");
  EXPECT_EQ(ronex_param_id, 0);

  //This serial number is not present on the param server -> returns -1
  ronex_param_id = get_ronex_param_id("1234");
  EXPECT_EQ( ronex_param_id, -1);

  std::string param;
  ros::param::set("/ronex/devices/0/product_id", "0x20001");
  ros::param::set("/ronex/devices/0/product_name", "general_io");
  ros::param::set("/ronex/devices/0/ronex_id", "my_beautiful_ronex");
  ros::param::set("/ronex/devices/0/path", "/ronex/general_io/my_beautiful_ronex/");
  ros::param::set("/ronex/devices/0/serial", "1234");

  //We now have a ronex with param id = 0 -> next free id is 1.
  ronex_param_id = get_ronex_param_id("");
  EXPECT_EQ(ronex_param_id, 1);

  //This ronex id is now present on the param server -> returns index = 0
  ronex_param_id = get_ronex_param_id("my_beautiful_ronex");
  EXPECT_EQ( ronex_param_id, 0);

  //This ronex id doesn't exist - return -1
  ronex_param_id = get_ronex_param_id("do_not_exist");
  EXPECT_EQ( ronex_param_id, -1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init( std::map<std::string, std::string>(), "test_sr_ronex_utilities");
  return RUN_ALL_TESTS();
}

/* For the emacs weenies in the crowd.
Local Variables:
c-basic-offset: 2
End:
*/
