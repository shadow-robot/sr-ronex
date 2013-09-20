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
def dimLED():
    # Set the switching frequency to 100kHz.
    pwm_period = 320
    # Start with a 100% duty cycle.
    pwm_on_time_0 = pwm_period
    # The second output is not used.
    pwm_on_time_1 = 0

    pub = rospy.Publisher( 'chatter', PWM )
    rospy.init_node('sr_ronex_dim_LED_with_PWM')
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)

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
This example demonstates how to dim a LED light with pulse-width modulation (PWM). 
"""
if __name__ == "__main__":
    try:
        dimLED()
    except rospy.ROSInterruptException:
        pass

    ronex_id = "1"
    example = SrRonexExample( ronex_id )
    path = example.get_ronex_path()
    
    if path != None:
        # For example "/ronex/general_io/1" + "/state"
        topic = path + "/state"
        rospy.Subscriber( topic, GeneralIOState, generalIOState_callback )
        rospy.spin()
    else:
        rospy.logerr( "Failed to find the ronex with the given ronex_id %s.", ronex_id )

#--------------------------------------------------------------------------------
