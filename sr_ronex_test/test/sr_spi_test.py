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
import rospy
import sys
import getopt
import unittest
import rostest
import os
from sr_ronex_msgs.srv import SPI
from sr_ronex_msgs.srv import SPIRequest
from sr_ronex_msgs.msg import SPIState
from std_msgs.msg import Bool
from threading import Lock
from numpy import uint8

from time import sleep


class TestSPIWithHardware(unittest.TestCase):
    '''
    A class used to test the Shadow Robot SPI HW.
    '''

    def setUp(self):
        self.find_spi_ronexes()

        self.controllers_list = ["ronex_" + ronex_id + "_passthrough" for ronex_id in self.ronex_ids]

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

        self.ronex_ids = [self.ronex_devs[0]['ronex_id']]

    def generalIOState_callback(self, data):
        # Note that the type of data.analogue is tuple.
        self.analogue_in = data.analogue_in

    def init_service_servers(self):
        # testing first spi ronex connected
        ronex = "/ronex/spi/" + str(self.ronex_devs[0]["ronex_id"])
        self.spi_srv = [rospy.ServiceProxy(ronex + "/command/passthrough/" + str(i), SPI) for i in xrange(4)]
        self.dyn_rcf_client = dynamic_reconfigure.client.Client(ronex)
        # subscribing to analogue input topic
        topic = ronex + "/state"
        rospy.Subscriber(topic, SPIState, self.generalIOState_callback)


############################################
# TEST START
############################################

    def test_if_controllers_are_loaded(self):
        list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        available_controllers = list_controllers()

        for ctrl in available_controllers.controller:
            self.assertTrue(ctrl.name in self.controllers_list,
                            msg="Available controllers: " + str(available_controllers.controller) +
                                " / expected controller: " + str(self.controllers_list))

    def test_all_cases(self):

        # Test 1, turn on all the digital outputs
        self.run_test_case([True, True, True, True, True, True],
                           [[3947, 3942, 3180, 3941], [3951, 3940, 3180, 3940]], [3563, 3542, 3540, 3535, 3544, 3761])
        # Test 2, turn off DIO_0
        self.run_test_case([False, True, True, True, True, True],
                           [[3950, 3830, 2415, 3941], [3951, 327, 3180, 3831]], [3563, 3542, 3540, 3438, 3445, 671])
        # Test 3, turn off DIO_1
        self.run_test_case([True, False, True, True, True, True],
                           [[3949, 326, 2415, 3941], [3951, 3833, 3188, 3831]], [3544, 3546, 3545, 3439, 310, 3666])
        # Test 4, turn off DIO_2
        self.run_test_case([True, True, False, True, True, True],
                           [[3950, 3832, 2415, 3943], [3949, 3833, 3187, 325]], [3547, 3546, 3545, 310, 3445, 3668])
        # Test 5, turn off DIO_3
        self.run_test_case([True, True, True, False, True, True],
                           [[3842, 3942, 3180, 327], [3842, 3945, 2423, 3942]], [3449, 3449, 312, 3534, 3542, 3762])
        # Test 6, turn off DIO_4
        self.run_test_case([True, True, True, True, False, True],
                           [[3842, 3944, 3181, 3831], [322, 3943, 2420, 3941]], [3450, 306, 3442, 3534, 3544, 3758])
        # Test 7, turn off DIO_5
        self.run_test_case([True, True, True, True, True, False],
                           [[323, 3943, 3180, 3831], [3843, 3943, 2419, 3942]], [308, 3446, 3445, 3536, 3542, 3761])

    def run_test_case(self, digital_states, expected_as, expected_analogue):
        success = True
        self.set_DIO_states(digital_states)
        # check channel 0 of all the spi modules
        results = []
        for adc_number in range(len(self.spi_srv)):
            results.append(self.read_adc(adc_number, 0))
            self.assertAlmostEquals(results[adc_number], expected_as[0][adc_number],
                                    msg="Testing channel 0 of " + str(adc_number) + "failed (delta = " +
                                        str(results[adc_number] - expected_as[0][adc_number]) + " / received = " +
                                        str(results[adc_number]) + ").", delta=400)

        # check channel 1 of all the spi modules
        results = []
        for adc_number in range(len(self.spi_srv)):
            results.append(self.read_adc(adc_number, 1))
            self.assertAlmostEquals(results[adc_number], expected_as[1][adc_number],
                                    msg="Testing channel 1 of " + str(adc_number) + "failed (delta = " +
                                        str(results[adc_number] - expected_as[1][adc_number]) + " / received = " +
                                        str(results[adc_number]) + ").", delta=400)

        # check all the analogue inputs
        for analogue_id in range(0, 6):
            self.assertAlmostEquals(self.analogue_in[analogue_id], expected_analogue[analogue_id],
                                    msg="Testing analogue input" + str(analogue_id) + "failed (delta = " +
                                        str(self.analogue_in[analogue_id] - expected_analogue[analogue_id]) +
                                        " / received = " + str(self.analogue_in[analogue_id]) + ").", delta=400)

    def set_DIO_states(self, digital_states):
        # set pre / post states
        if len(digital_states) == 6:
            cfg = {"pin_output_state_pre_DIO_0": digital_states[0], "pin_output_state_post_DIO_0": digital_states[0],
                   "pin_output_state_pre_DIO_1": digital_states[1], "pin_output_state_post_DIO_1": digital_states[1],
                   "pin_output_state_pre_DIO_2": digital_states[2], "pin_output_state_post_DIO_2": digital_states[2],
                   "pin_output_state_pre_DIO_3": digital_states[3], "pin_output_state_post_DIO_3": digital_states[3],
                   "pin_output_state_pre_DIO_4": digital_states[4], "pin_output_state_post_DIO_4": digital_states[4],
                   "pin_output_state_pre_DIO_5": digital_states[5], "pin_output_state_post_DIO_5": digital_states[5]}
        else:
            cfg = {"pin_output_state_pre_DIO_0": True, "pin_output_state_post_DIO_0": True,
                   "pin_output_state_pre_DIO_1": True, "pin_output_state_post_DIO_1": True,
                   "pin_output_state_pre_DIO_2": True, "pin_output_state_post_DIO_2": True,
                   "pin_output_state_pre_DIO_3": True, "pin_output_state_post_DIO_3": True,
                   "pin_output_state_pre_DIO_4": True, "pin_output_state_post_DIO_4": True,
                   "pin_output_state_pre_DIO_5": True, "pin_output_state_post_DIO_5": True}
        self.dyn_rcf_client.update_configuration(cfg)

    def read_adc(self, adc_number, channel_number):
        req = SPIRequest()
        # set data
        req.data = [0x0]*3
        req.data[0] = 0x01
        if channel_number == 0:
            req.data[1] = 0xA0
        else:
            req.data[1] = 0xE0
        req.data[2] = 0x01
        spi_res = None
        # The SPI response is generated only after the first packet is
        # sent so we need to send a message twice.
        for i in range(2):
            try:
                spi_res = self.spi_srv[adc_number](req)
            except rospy.ServiceException as exc:
                self.assertTrue(False, "Failed to send data to "+str(adc_number) +
                                " channel "+str(channel_number)+" -> " + str(exc))
        result = int(spi_res.data[1])
        result <<= 8
        result |= int(spi_res.data[2])
        result &= 0x0FFF
        return result

############################################
# TEST END
############################################

if __name__ == '__main__':
    rospy.init_node('sr_spi_test')
    rostest.rosrun('sr_spi_test', 'sr_spi_test', TestSPIWithHardware)
