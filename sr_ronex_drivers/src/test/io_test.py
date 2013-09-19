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

import roslib; roslib.load_manifest('sr_ronex_drivers')
import rospy
import sys, getopt

from threading import Lock
from sr_ronex_msgs.msg import BoolArray
from std_msgs.msg import UInt16MultiArray

PRODUCT_CODE = "0x05300424"
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
    For the PRODUCT_CODE = "0x05300424"
    4 digital inputs/outputs/PWM outputs
    4 analog inputs
    2 analog outputs

    The digital outputs will be tested by turning on and off LEDs attached to them.
    The digital inputs will read back the value we set for the outputs.
    The PWM outputs will be set to their lower frequency, and the digital inputs used to check the on-off period
    The analog output 0 will be wired to analog inputs 0 and 2, output 1 to inputs 1 and 3

    Keep in mind that the test is intended to be run with the LEDs attached to the digital I/Os and the correct wiring of the analog I and O
    """
#Example of existing topics for a certain device
#/device_0x05300424_0x00000016_PWM_outputs_command
#/device_0x05300424_0x00000016_analog_inputs_state
#/device_0x05300424_0x00000016_analog_outputs_command
#/device_0x05300424_0x00000016_digital_inputs_state
#/device_0x05300424_0x00000016_digital_outputs_command

    def __init__(self, device_SN):
        rospy.loginfo("Testing device. Product code: " + PRODUCT_CODE + " SN: " + device_SN)
        self.success = True
        self.PWM_testing = False
        self.device_SN = device_SN
        self.a_state_lock = Lock()
        self.d_state_lock = Lock()
        self.analog_command_publisher = rospy.Publisher("/device_" + PRODUCT_CODE + "_" + device_SN + "_analog_outputs_command", UInt16MultiArray, latch=True)
        self.digital_command_publisher = rospy.Publisher("/device_" + PRODUCT_CODE + "_" + device_SN + "_digital_outputs_command", BoolArray, latch=True)
        self.PWM_command_publisher = rospy.Publisher("/device_" + PRODUCT_CODE + "_" + device_SN + "_PWM_outputs_command", UInt16MultiArray, latch=True)
        self.analog_state_subscriber_ = rospy.Subscriber("/device_" + PRODUCT_CODE + "_" + device_SN + "_analog_inputs_state", UInt16MultiArray, self.analog_state_callback)
        self.digital_state_subscriber_ = rospy.Subscriber("/device_" + PRODUCT_CODE + "_" + device_SN + "_digital_inputs_state", BoolArray, self.digital_state_callback)

        self.last_analog_state = None
        self.last_digital_state = None
        self.last_period_start_time = [0.0, 0.0, 0.0, 0.0]
        self.average_period = [0.0, 0.0, 0.0, 0.0]

        #Initialize the PWM outputs to 0 to avoid interference with the digital output testing
        command_msg = UInt16MultiArray(None, [0, 0, 0, 0, 0, 0, 0, 0])
        self.PWM_command_publisher.publish(command_msg)
        rospy.sleep(0.2)
       
        
        
    def analog_state_callback(self, msg):
        with self.a_state_lock:
            self.last_analog_state = msg

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
                rospy.logerr("No digital input data recived from: " + self.device_SN)
                self.success = False

    def test_digital_io_case(self, output_values):
        command_msg = BoolArray(output_values)
        self.digital_command_publisher.publish(command_msg)
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

    def check_analog_inputs(self, output_values):
        rospy.sleep(0.1)
        with self.a_state_lock:
            if self.last_analog_state != None:
                for i, value in enumerate(output_values):
                    index = i
                    if (float(value) / float(self.last_analog_state.data[index])) > ANALOG_RATIO_UPPER or \
                       (float(value) / float(self.last_analog_state.data[index])) < ANALOG_RATIO_LOWER:
                        rospy.logerr("Wrong value in analog input " + str(index) + " set: " + str(value) + " measured: " + str(self.last_analog_state.data[index]) + " ratio: " + str(float(value) / float(self.last_analog_state.data[index])))
                        self.success = False
                    index = i + 2
                    if (float(value) / float(self.last_analog_state.data[index])) > ANALOG_RATIO_UPPER or \
                       (float(value) / float(self.last_analog_state.data[index])) < ANALOG_RATIO_LOWER:
                        rospy.logerr("Wrong value in analog input " + str(index) + " set: " + str(value) + " measured: " + str(self.last_analog_state.data[index]) + " ratio: " + str(float(value) / float(self.last_analog_state.data[index])))
                        self.success = False
            else:
                rospy.logerr("No analog input data recived from: " + self.device_SN)
                self.success = False	


    def test_analog_io_case(self, output_values):
        command_msg = UInt16MultiArray(None, output_values)
        self.analog_command_publisher.publish(command_msg)
        self.check_analog_inputs(output_values)
        rospy.sleep(ANALOG_IO_WAIT)

    def test_analog_ios(self):
        rospy.loginfo("Testing analog I/O")
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
        rospy.loginfo("Analog I/O test ended")

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
            

    def run_test(self):
        self.test_digital_ios()
        self.test_analog_ios()
        self.test_PWM_outputs()
        #self.ramp()
        if self.success:
            rospy.loginfo("NO ERRORS DETECTED. Product code: " + PRODUCT_CODE + " SN: " + self.device_SN)
        else:
            rospy.logerr("TEST FAILED. ERRORS DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!! Product code: " + PRODUCT_CODE + " SN: " + self.device_SN)

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"h")
    except getopt.GetoptError:
        print 'io_test.py <SN_device_1> ... <SN_device_N>'
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print 'io_test.py <SN_device_1> ... <SN_device_N>'
            sys.exit()

    if len(args) == 0:
       print 'io_test.py <SN_device_1> ... <SN_device_N>'
       sys.exit(2)

    # init the ros node
    rospy.init_node('IO_test', anonymous=True)
    rospy.sleep(0.5)
 
    for arg in args:
        io_test = IoTest(arg)
        io_test.run_test()

    rospy.loginfo("I/O test ended")
    # subscribe until interrupted
    #rospy.spin()



if __name__ == '__main__':
    main(sys.argv[1:])


