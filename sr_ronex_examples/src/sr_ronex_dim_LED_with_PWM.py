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

import roslib; roslib.load_manifest('sr_ronex_examples')
import rospy
from time import sleep
from sr_ronex_msgs.msg import PWM

#--------------------------------------------------------------------------------

# Dim a LED light with PWM. It takes 10 seconds.
def dimLED(topic):
    # Set the switching frequency to 100kHz.
    pwm_period = 320
    # Start with a 100% duty cycle.
    pwm_on_time_0 = pwm_period
    # The second output is not used.
    pwm_on_time_1 = 0

    pub = rospy.Publisher( topic, PWM )
    while not rospy.is_shutdown():
        # Dim the light...
        pwm_on_time_0 -= 3
        if pwm_on_time_0 < 0:
            pwm_on_time_0 = pwm_period

        # Set the PWM data.
        pwm = PWM()
        pwm.pwm_period    = pwm_period
        pwm.pwm_on_time_0 = pwm_on_time_0
        pwm.pwm_on_time_1 = pwm_on_time_1
        
        pub.publish( pwm )
        rospy.sleep( 0.1 )

#--------------------------------------------------------------------------------

"""
This class demonstate how to read the analog data for a given ronex.
"""
class SrRonexExample(object):

    def __init__(self, ronex_id):
        self.ronex_id = ronex_id
        
    def get_ronex_path(self):
        """
        Find the ronexes present on the system.
        """
        # Wait until there's one ronex.
        while True:
            try:
                rospy.get_param("/ronex/devices/0/ronex_id")
                break
            except:
                rospy.loginfo("Waiting for the ronex to be loaded properly.")
                sleep(0.1)

        # Retreive all the ronex ids from the parameter server.
        ronex_param = rospy.get_param("/ronex/devices")
        for key in ronex_param:
            if self.ronex_id == ronex_param[key]["ronex_id"]:
                path = ronex_param[key]["path"]
                return path

#--------------------------------------------------------------------------------

"""
This example demonstates how to dim a LED light with pulse-width modulation (PWM). 
"""
if __name__ == "__main__":
    rospy.init_node('sr_ronex_dim_LED_with_PWM')
    
    ronex_id = "1"
    example = SrRonexExample( ronex_id )
    path = example.get_ronex_path()
     
    if path != None:
        # Use the first PWM output to dim the LED light.
        # For example "/ronex/general_io/1" + "/command/0".
        topic = path + "/command/0"
        try:
            dimLED(topic)
        except rospy.ROSInterruptException:
            pass

#--------------------------------------------------------------------------------
