#!/usr/bin/env python
"""
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
"""

import rospy, sys, getopt, unittest, rostest
import dynamic_reconfigure.client

from time import sleep
from string import Template
from threading import Lock
from sr_ronex_msgs.msg import BoolArray
from std_msgs.msg import UInt16MultiArray

DIGITAL_IO_WAIT = 0.5
ANALOG_IO_WAIT = 0.1
PWM_O_WAIT = 3.0
ANALOG_RATIO_UPPER = 1.4
ANALOG_RATIO_LOWER = 1.0
PERIOD_ALPHA = 0.01
MAX_PWM_PERIOD = 0.008
MIN_PWM_PERIOD = 0.005

class IoTest(object):
    """
    A class used to test the Shadow Robot ethercat board HW.
    
    For the test to succeed 2 ronex modules are required
    each with a stacker on it
    The digital I/O of one module should be connected to the I/O of the other
    The analogue I/O of one of the modules should be connected to a constant voltage source
    
    On each module
    12 digital inputs/outputs
    6 PWM outputs
    12 analogue inputs

    The digital inputs will read back the value we set for the outputs.
    Then all the I/O should switch roles from Input to Output and vice versa
    The PWM outputs will be set to their lower frequency, and the digital inputs used to check the on-off period
    The analogue inputs should be wired to an appropriate constant voltage source
    """

    def __init__( self, devs ):
        rospy.loginfo("Creating test class")
        self.success = True
        self.a_state_lock = Lock()
        self.d_state_lock = Lock()
        self.client = dynamic_reconfigure.client.Client("ronex")
        params = { 'my_string_parameter' : 'value', 'my_int_parameter' : 5 }
        config = client.update_configuration(params)
        
        sdt = Template("/ronex/general_io/$sn/command/digital")
        snA = devs['0']["serial"]
        snB = devs['1']["serial"]
        self.digital_publisher_A = rospy.Publisher( sdt.substitute( sn = snA ), BoolArray, latch = True )
        self.digital_publisher_B = rospy.Publisher( sdt.substitute( sn = snB ) , BoolArray, latch = True )

        self.digital_subscriber_A = rospy.Subscriber( sdt.substitute( sn = snA ), BoolArray, self.digital_state_callback )
        self.digital_subscriber_B = rospy.Subscriber( sdt.substitute( sn = snB ), BoolArray, self.digital_state_callback )
        
        
        self.last_digital_state = None
        self.last_period_start_time = 4*[0.0]
        self.average_period = 4*[0.0]

        #Initialise the PWM outputs to 0 to avoid interference with the digital output testing
        command_msg = UInt16MultiArray(None, 8*[0])
        #self.PWM_command_publisher.publish(command_msg)
        rospy.sleep(0.2)

    def digital_state_callback(self, msg):
        with self.d_state_lock:
            if self.PWM_testing:
                for i, value in enumerate(self.last_digital_state.data):
                    #detect the falling edge of the PWM signal
                    if value and not msg.data[i]:
                        if self.last_period_start_time[i] != 0.0:
                            self.average_period[i] = (1- PERIOD_ALPHA) * self.average_period[i] + PERIOD_ALPHA * (rospy.get_time() - self.last_period_start_time[i])
                        self.last_period_start_time[i] = rospy.get_time()
            self.last_digital_state = msg

    def check_digital_inputs(self, output_values):
        rospy.sleep(0.1)
        with self.d_state_lock:
            if self.last_digital_state != None:
                for i, value in enumerate(output_values):
                    if i & 1:
                        if self.last_digital_state.data[i/2] != value:
                            rospy.logerr("Wrong value in digital input " + str(i/2))
                            self.success = False
            else:
                rospy.logerr("No digital input data received")
                self.success = False

    def test_digital_io_case(self, output_values):
        command_msg = BoolArray(output_values)
        self.digital_publisher_A.publish(command_msg)
        self.check_digital_inputs(output_values)
        rospy.sleep(DIGITAL_IO_WAIT)

    def test_digital_ios(self):
        rospy.loginfo("Testing digital I/O")
        self.test_digital_io_case([False, False, False, False, False, False, False, False])
        self.test_digital_io_case([False, True, False, False, False, False, False, False])
        self.test_digital_io_case([False, False, False, True, False, False, False, False])
        self.test_digital_io_case([False, False, False, False, False, True, False, False])
        self.test_digital_io_case([False, False, False, False, False, False, False, True])
        self.test_digital_io_case([False, True, False, True, False, True, False, True])
        self.test_digital_io_case([False, False, False, False, False, False, False, False])
        rospy.loginfo("Digital I/O test ended")

class TestContainer(unittest.TestCase):
    def test_connected_ronex(self):
        ronex_devices = find_ronexes()
        self.assertEqual( len(ronex_devices), 2, "Error. Connect a ronex bridge with 2 ronex devices" )
            
        io_test = IoTest( ronex_devices )
        
        io_test.test_digital_ios()
        self.assertTrue( io_test.success, "digital I/O test failed" )
        
        #io_test.test_analog_ios()
        #self.assertTrue( io_test.success, "analogue I/O test failed" )
        
        #io_test.test_PWM_ios()
        #self.assertTrue(io_test.success, "PWM I/O test failed")
        
def find_ronexes():
    attempts = 50
    while attempts:
        try:
            rospy.get_param("/ronex/devices/0/ronex_id")
            break
        except:
            rospy.loginfo("Waiting for the ronex to be loaded properly.")
            sleep(0.1)
            attempts -= 1
    return rospy.get_param("/ronex/devices")
        
if __name__ == '__main__':

    rospy.init_node('single_ronex')
    sleep(0.5)
    rostest.rosrun('sr_ronex_tests', 'single_ronex', TestContainer )
