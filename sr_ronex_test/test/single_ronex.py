#!/usr/bin/env python
"""
 Copyright 2013 Shadow Robot Company Ltd.

 This program is Proprietary software: you cannot redistribute it or modify it

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.
"""

import rospy, sys, getopt, unittest, rostest

from time import sleep
from string import Template
from threading import Lock
from sr_ronex_msgs.msg import BoolArray
from std_msgs.msg import UInt16MultiArray

PRODUCT_CODE = "0X02000000"
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
    For the PRODUCT_CODE = "0X02000000"
    4 digital inputs/outputs/PWM outputs
    4 analogue inputs
    2 analogue outputs

    The digital outputs will be tested by turning on and off LEDs attached to them.
    The digital inputs will read back the value we set for the outputs.
    The PWM outputs will be set to their lower frequency, and the digital inputs used to check the on-off period
    The analogue output 0 will be wired to analogue inputs 0 and 2, output 1 to inputs 1 and 3

    Keep in mind that the test is intended to be run with the LEDs attached to the digital I/Os and the correct wiring of the analogue I and O
    """

    def __init__( self, devs ):
        rospy.loginfo("Testing device. Product code: " + PRODUCT_CODE )
        self.success = True
        self.PWM_testing = False
        self.a_state_lock = Lock()
        self.d_state_lock = Lock()
        
        st = Template("/ronex/general_io/$sn/command/digital")
        snA = devs['0']["serial"]
        snB = devs['1']["serial"]
        self.digital_publisher_A = rospy.Publisher( st.substitute( sn = snA ), BoolArray, latch = True )
        self.digital_publisher_B = rospy.Publisher( st.substitute( sn = snB ) , BoolArray, latch = True )
        self.digital_subscriber_A = rospy.Subscriber( st.substitute( sn = snA ), BoolArray, self.digital_state_callback )
        self.digital_subscriber_B = rospy.Subscriber( st.substitute( sn = snB ), BoolArray, self.digital_state_callback )
        
        self.last_analog_state = None
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
"""
    def analog_state_callback(self, msg):
        with self.a_state_lock:
            self.last_analog_state = msg

    def check_analog_inputs(self, output_values):
        rospy.sleep(0.1)
        with self.a_state_lock:
            if self.last_analog_state != None:
                for i, value in enumerate(output_values):
                    index = i
                    if (float(value) / float(self.last_analog_state.data[index])) > ANALOG_RATIO_UPPER or \
                       (float(value) / float(self.last_analog_state.data[index])) < ANALOG_RATIO_LOWER:
                        rospy.logerr("Wrong value in analogue input " + str(index) + " set: " + str(value) + " measured: " + str(self.last_analog_state.data[index]) + " ratio: " + str(float(value) / float(self.last_analog_state.data[index])))
                        self.success = False
                    index = i + 2
                    if (float(value) / float(self.last_analog_state.data[index])) > ANALOG_RATIO_UPPER or \
                       (float(value) / float(self.last_analog_state.data[index])) < ANALOG_RATIO_LOWER:
                        rospy.logerr("Wrong value in analogue input " + str(index) + " set: " + str(value) + " measured: " + str(self.last_analog_state.data[index]) + " ratio: " + str(float(value) / float(self.last_analog_state.data[index])))
                        self.success = False
            else:
                rospy.logerr("No analogue input data received from: " + self.device_SN)
                self.success = False

    def test_analog_io_case(self, output_values):
        command_msg = UInt16MultiArray(None, output_values)
        self.analog_command_publisher.publish(command_msg)
        self.check_analog_inputs(output_values)
        rospy.sleep(ANALOG_IO_WAIT)

    def test_analog_ios(self):
        rospy.loginfo("Testing analogue I/O")
        rospy.sleep(0.1)
        self.test_analog_io_case([0x200, 0x200])
        self.test_analog_io_case([0x200, 0xFF00])
        self.test_analog_io_case([0xFF00, 0x200])
        self.test_analog_io_case([0x200, 0x200])
        self.test_analog_io_case([0x300, 0x300])
        self.test_analog_io_case([0x1000, 0x1000])
        self.test_analog_io_case([0x5000, 0x5000])
        self.test_analog_io_case([0x8000, 0x8000])
        self.test_analog_io_case([0xA000, 0xA000])
        self.test_analog_io_case([0xC000, 0xC000])
        self.test_analog_io_case([0xD000, 0xD000])
        self.test_analog_io_case([0xE000, 0xE000])
        self.test_analog_io_case([0xF000, 0xF000])
        self.test_analog_io_case([0xFF00, 0xFF00])
        rospy.loginfo("Analogue I/O test ended")

    def test_PWM_o_case(self, output_values):
        command_msg = UInt16MultiArray(None, output_values)
        self.PWM_testing = True
        self.PWM_command_publisher.publish(command_msg)
        #self.check_digital_inputs(output_values)
        rospy.sleep(PWM_O_WAIT)
        for i, value in enumerate(self.average_period):
            if value > MAX_PWM_PERIOD or value < MIN_PWM_PERIOD:
                rospy.logerr("Wrong period in PWM output " + str(i) + " measured period: " + str(value))
                self.success = False
        self.PWM_testing = False

    def test_PWM_outputs(self):
        rospy.loginfo("Testing PWM outputs")
        self.test_PWM_o_case([0xFFFE, 0x8000, 0xFFFE, 0x8000, 0xFFFE, 0x8000, 0xFFFE, 0x8000])
        rospy.loginfo("PWM outputs test ended")


    def ramp(self):
        for i in range(0, 65535, 0x100):
            command_msg = UInt16MultiArray(None, [i, i])
            self.analog_command_publisher.publish(command_msg)
            rospy.sleep(0.03)
"""
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

    rospy.init_node('single_ronex', anonymous=True)
    sleep(0.5)
    rostest.rosrun('sr_ronex_tests', 'single_ronex', TestContainer )
