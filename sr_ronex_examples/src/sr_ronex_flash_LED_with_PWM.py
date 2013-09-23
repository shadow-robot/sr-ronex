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

# Flash a LED light with PWM.
def flashLED(topic):
    pwm_period = 320
    # Start with a 100% duty cycle.
    pwm_on_time_0 = pwm_period
    # The second output is not used.
    pwm_on_time_1 = 0

    pub = rospy.Publisher( topic, PWM )
    while not rospy.is_shutdown():
        # Flash the light...
        pwm_on_time_0 -= 10
        if pwm_on_time_0 < 0:
            pwm_on_time_0 = pwm_period

        # Set the PWM data.
        pwm = PWM()
        pwm.pwm_period    = pwm_period
        pwm.pwm_on_time_0 = pwm_on_time_0
        pwm.pwm_on_time_1 = pwm_on_time_1

        pub.publish( pwm )
        rospy.sleep( 0.01 )

#--------------------------------------------------------------------------------

"""
This class demonstrates how to find the General I/O module with the given ronex_id.
"""
class SrRonexFindGeneralIOModule(object):

    def __init__(self, ronex_id):
        self.ronex_id = ronex_id
        
    """
    Get the path of the General I/O module with the given ronex_id.
    Note that None is returned if the module is not found.
    """
    def get_path(self):
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

        """
        Retrieve all the ronex parameter ids from the parameter server.
        If there are three General I/O modules, then ronex_param_ids is [0,1,2].
        Note that the id starts from zero. And the size of the returned variable
        is equal to the number of General I/O modules.
        """
        ronex_param_ids = rospy.get_param("/ronex/devices")
        for key in ronex_param_ids:
            if self.ronex_id == ronex_param_ids[key]["ronex_id"]:
                path = ronex_param_ids[key]["path"]
                return path

#--------------------------------------------------------------------------------

"""
Assume that your RoNeX consists of a Bridge (IN) module, and one or multiple General I/O module(s).
This example demonstrates how to flash a LED light with pulse-width modulation (PWM). 
"""
if __name__ == "__main__":
    rospy.init_node('sr_ronex_flash_LED_with_PWM')
    
    # Note that you may have to set the value of ronex_id,
    # depending on which General I/O board the LED is connected to.
    ronex_id = raw_input( "Please enter the ronex id: " )
    findModule = SrRonexFindGeneralIOModule( str(ronex_id) )
    path = findModule.get_path()
     
    if path != None:
        # Always use the first digital I/O channel to flash the LED light.
        # For example "/ronex/general_io/1" + "/command/pwm/0".
        topic = path + "/command/pwm/0"
        rospy.loginfo("topic = %s.", topic)
        try:
            flashLED(topic)
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.loginfo("Failed to find the General I/O module with the given ronex_id %s.", ronex_id)

#--------------------------------------------------------------------------------
