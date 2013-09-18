#!/usr/bin/env python

"""
Copyright 2013 Shadow Robot Company Ltd.

This program is Proprietary software: you cannot redistribute it or modify it

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.
"""

import roslib
import rospy
from time import sleep

class SR_RONEX_Example(object):

    def __init__(self):
        """
        """
        ronex_ids = self.find_ronexes()
        self.set_param(ronex_ids)
        
    def find_ronexes(self):
        """
        Find the ronexes present on the system

        @return a list of ronex_ids
        """
        ronex_ids = []

        #retreive all the ronex ids from the parameter server
        #wait until there's one ronex
        while True:
            try:
                rospy.get_param("/ronex/0/ronex_id")
                sleep(0.1)
            except:
                rospy.loginfo("Waiting for the ronex to be loaded properly.")
                pass
            break

        ronex_param = rospy.get_param("/ronex")
        for key in ronex_param:
            ronex_ids.append(ronex_param[key]["ronex_id"])

        return ronex_ids

    def set_param(self, ronex_ids):
        """
        Load the parameters for the present ronexes into the parameter server

        @param ronex_ids the ids of the ronexes
        """
        for ronex_id in ronex_ids:
            rospy.set_param("/ronex_"+ronex_id+"_passthrough/type", "sr_ronex_controllers/GeneralIOPassthroughController")
            rospy.set_param("/ronex_"+ronex_id+"_passthrough/ronex_id", ronex_id)


if __name__ == "__main__":
    rospy.init_node("sr_ronex_parse_parameter_servers")
    SR_RONEX_Example()
