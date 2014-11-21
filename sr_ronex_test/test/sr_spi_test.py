#!/usr/bin/env python
# ####################################################################
# Copyright (c) 2013, Shadow Robot Company, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.
# ####################################################################

from controller_manager_msgs.srv import ListControllers
import dynamic_reconfigure.client
import rospy, sys, getopt, unittest, rostest, os
from sr_ronex_msgs.srv import SPI
from sr_ronex_msgs.srv import SPIRequest
from std_msgs.msg import Bool
from threading import Lock
from numpy import uint8


# from time import sleep
class TestSPIWithHardware(unittest.TestCase):
  '''
  A class used to test the Shadow Robot SPI HW.
  '''

  def setUp(self):
    self.find_spi_ronexes()

    self.controllers_list = [ "ronex_" + ronex_id + "_passthrough" for ronex_id in self.ronex_ids ]

    self.init_service_servers()
    self.state_lock = Lock()

    rospy.sleep(0.4)  # wait for self.state to be updated for the first time

  def tearDown(self):
    pass

  def find_spi_ronexes(self):
    attempts = 50
    rospy.loginfo('Waiting for the ronex to be loaded properly.')
    while attempts and not rospy.has_param('/ronex/devices/0/ronex_id'):
      rospy.sleep(0.1)
      attempts -= 1

    self.assertNotEqual(attempts, 0, 'Failed to get ronex devices from parameter server')

    all_ronexes = rospy.get_param('/ronex/devices')
    self.ronex_devs = []
    for i in all_ronexes:
      if "spi" == all_ronexes[i]["product_name"]:
        self.ronex_devs.append(all_ronexes[i])

    self.assertEqual(len(self.ronex_devs), 1, 'Error. Connect a ronex bridge with at least 1 spi module.')

    self.ronex_ids = [ self.ronex_devs[0]['ronex_id'] ]


  def init_service_servers(self):
    #testing first spi ronex connected
    ronex = "/ronex/spi/" + str(self.ronex_devs[0]["ronex_id"])
    self.spi_srv = [ rospy.ServiceProxy(ronex + "/command/passthrough/" + str(i), SPI) for i in xrange(4)]
    self.dyn_rcf_client = dynamic_reconfigure.client.Client(ronex)


############################################
# TEST START
############################################

  def test_if_controllers_are_loaded(self):
    list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
    available_controllers = list_controllers()

    for ctrl in available_controllers.controller:
      self.assertTrue(ctrl.name in self.controllers_list, msg="Available controllers: "+str(available_controllers.controller) + " / expected controller: " +str(self.controllers_list) )

  def test_all_cases(self):
    # in the document
    # expected_A0, expected_A1, expected_A2, expected_A3,
    # expected_A4, expected_A5, expected_PA, expected_PD)
    #
    # we expect in the function:
    # expected: [[A5 A1 PD A3][A4 P0 PA A2]]

    #Test 1
    self.run_test_case([True, True, True, True, True],
                       [[2048, 2048, 2048, 2048], [2048, 2048, 2048, 2048]])

    #Test 2
    self.run_test_case([False, True, True, True, True],)

  def run_test_case(self, digital_states, expected_As):
    success = True

    # expected: A5 A1 PD A3
    results = []
    for adc_number in range(len(self.spi_srv)):
      results.append( self.read_adc(adc_number, 0) )

    self.assertAlmostEquals(results, expected_As[0], msg="Testing channel 0 failed.", delta=30)

    # expected: A4 P0 PA A2
    results = []
    for adc_number in range(len(self.spi_srv)):
      results.append( self.read_adc(adc_number, 1) )

    self.assertAlmostEquals(results, expected_As[1], msg="Testing channel 1 failed.", delta=30)

  def read_adc(self, adc_number, channel_number):
    req = SPIRequest()

    # set pre / post states
    cfg = {"pin_output_state_pre_DIO_0": True, "pin_output_state_post_DIO_0": False,
           "pin_output_state_pre_DIO_1": True, "pin_output_state_post_DIO_1": False,
           "pin_output_state_pre_DIO_2": True, "pin_output_state_post_DIO_2": False,
           "pin_output_state_pre_DIO_3": True, "pin_output_state_post_DIO_3": False,
           "pin_output_state_pre_DIO_4": True, "pin_output_state_post_DIO_4": False,
           "pin_output_state_pre_DIO_5": True, "pin_output_state_post_DIO_5": False}

    cfg["pin_output_state_pre_DIO_" + str(adc_number)] = False
    cfg["pin_output_state_post_DIO_" + str(adc_number)] = True

    self.dyn_rcf_client.update_configuration(cfg)

    # set data
    req.data = [0x0]*3
    req.data[0] = 0x01
    if channel_number != 0:
      req.data[1] = 0xA0
    else:
      req.data[1] = 0xB0
    req.data[2] = 0x01

    spi_res = None
    # The SPI respons is generated only after the first packet is
    # sent so we need to send a message twice.
    for i in range(2):
      try:
        spi_res = self.spi_srv[adc_number](req)
      except rospy.ServiceException as exc:
        self.assertTrue(False, "Failed to send data to "+str(adc_number) +
                        " channel "+str(channel_number)+" -> "+ str(exc)   )

    result = hex(spi_res.data[1])
    result <<= 8
    result |= hex(spi_res.data[2])
    result &= 0x0FFF
    return result

############################################
# TEST END
############################################

if __name__ == '__main__':
    rospy.init_node('sr_spi_test')
    rostest.rosrun('sr_spi_test', 'sr_spi_test', TestSPIWithHardware)
