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


class TestSPIWithHardware():
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
        
        if attempts is 0:
            rospy.loginfo('Failed to get ronex devices from parameter server')

        all_ronexes = rospy.get_param('/ronex/devices')
        self.ronex_devs = []
        for i in all_ronexes:
            if "spi" == all_ronexes[i]["product_name"]:
                self.ronex_devs.append(all_ronexes[i])

        if len(self.ronex_devs) != 1:
            rospy.loginfo('Error. Connect a ronex bridge with at least 1 spi module.')

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
        print available_controllers
        for ctrl in available_controllers.controller:
            if ctrl.name not in self.controllers_list:
                rospy.loginfo("Available controllers: " + str(available_controllers.controller) +
                    " / expected controller: " + str(self.controllers_list))
                return False
        
        return True

    def run_one_case(self, digital_states, expected_as):
        success = True
        self.set_DIO_states(digital_states)
        # check channel 0 of all the spi modules
        results = []
        for adc_number in range(len(self.spi_srv)):
            #adc_number = 1
            results.append(self.read_adc(adc_number, 0))
            rospy.loginfo("received = " + str(results))

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
        for i in range(1):
            try:
                spi_res = self.spi_srv[adc_number](req)
            except rospy.ServiceException as exc:
                rospy.loginfo("Failed to send data to "+str(adc_number) +
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
    spi_test = TestSPIWithHardware()
    spi_test.setUp()
    if spi_test.test_if_controllers_are_loaded():
        rospy.loginfo("Controllers loaded")
    else:
        rospy.loginfo("No Controllers loaded")
    for i in range(20):
        spi_test.run_one_case([False, True, True, True, True, True],
                              [[3947, 3942, 3180, 3941], [3951, 3940, 3180, 3940]])
