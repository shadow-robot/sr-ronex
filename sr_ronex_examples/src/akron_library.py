from rospy import ServiceProxy
from sr_ronex_msgs.srv import SPI

class AnalogReader(object):
    """
    retrieves values from analog to digital converters (ADCs)
    connnected to 6 spi modules
    """

    def __init__(self):
        # serials of ronex devices
        serials = ("1402573926", "1402574043")

        # passthrough controller names for each ronex device
        devices = ("/ronex/spi/" + serials[0] + "/command/passthrough/",
                   "/ronex/spi/" + serials[1] + "/command/passthrough/")

        # list of services for each SPI board ordered by ADC index
        self.proxies = (ServiceProxy(devices[0] + '0', SPI),
                        ServiceProxy(devices[0] + '1', SPI),
                        ServiceProxy(devices[1] + '0', SPI),
                        ServiceProxy(devices[1] + '1', SPI),
                        ServiceProxy(devices[1] + '2', SPI),
                        ServiceProxy(devices[1] + '3', SPI))

        # map analog channel to proxies (ADC)
        self.channel_map = { 0: 0,  1: 1,  2: 2,  3: 3,  4: 4,  5: 5,
                             8: 0,  9: 1, 10: 2, 11: 3, 12: 4, 13: 5,
                            16: 0, 17: 1, 18: 2, 19: 3, 20: 4, 21: 5,
                            24: 0, 25: 1, 26: 2, 27: 3, 28: 4, 29: 5}

        # request for ros services to get analog channel 0,1,2,3 of each ADC
        self.analog_requests = ([6,   0, 0],
                                [6,  64, 0],
                                [6, 128, 0],
                                [6, 192, 0])

        # map analog channel to analog_request index
        self.request_map = { 0: 0,  1: 0,  2: 0,  3: 0,  4: 0,  5: 0,
                             8: 1,  9: 1, 10: 1, 11: 1, 12: 1, 13: 1,
                            16: 2, 17: 2, 18: 2, 19: 2, 20: 2, 21: 2,
                            24: 3, 25: 3, 26: 3, 27: 3, 28: 3, 29: 3}

        # analog channels per ADC (1,2,3,4,5,6)
        self.channels = ((0,  8, 16, 24),
                         (1,  9, 17, 25),
                         (2, 10, 18, 26),
                         (3, 11, 19, 27),
                         (4, 12, 20, 28),
                         (5, 13, 21, 29))

        # limits of raw analog values
        self.raw_min =  400.0
        self.raw_max = 3500.0

        # desired range of analog readings
        self.min = -1.0
        self.max =  1.0

        # coefficient to normalize raw analog values to desired range
        self.raw_coefficient = (self.max - self.min) / (self.raw_max - self.raw_min)

    def analog_channel(self, channel):
        """
        get the current analog reading of given channel
        self.channels contains all valid values for channel
        """
        if not self.channel_map.has_key(channel):
            print("channel {} is out of range".format(channel))
            return

        request = self.analog_requests[self.request_map[channel]]
        response = self.proxies[self.channel_map[channel]](request).data

        # union of the high niblle of response[1] and all data in response[2]
        bin_value = '{0:08b}{1:08b}'.format(int(response[1]), int(response[2]))

        raw_value = int(bin_value[4:], 2)
        raw_value = min(max(raw_value, self.raw_min), self.raw_max)
        return self.raw_coefficient*(raw_value - self.raw_min) + self.min

    def analogs_by_adc(self, adc):
        """
        get list of current analog readings for all channels of ADC
        basically for each row of self.channels. Valid adc values are [1, 6]
        """
        if adc not in xrange(1, 7):
            print("adc {} out of range".format(adc))
            return
        return [self.analog_channel(ch) for ch in self.channels[adc - 1]]

    def all_analogs(self):
        """
        get list of all available analog readings
        """
        return [self.analog_channel(ch) for adc in self.channels for ch in adc]

from rospy import is_shutdown, init_node, Rate, Time
from sr_hand.shadowhand_commander import Commander
from math import sin, pi

class JointMotions(object):
    """
    Helps to execute some joint motions
    """
    def __init__(self):
        self.motion_allowed = True

        init_node("AkronJointMotions")
        self.commander = Commander()

        # scale coefficients from desired analog range to each joint's limits in degrees
        self.jnt_coeffs = {joint.name : (joint.max - joint.min) / (self.max - self.min)
                           for joint in self.commander.hand.allJoints}

        # offset to add after scaling analogs to joint range to start from joint's low limit
        self.jnt_offset = {joint.name : joint.min / self.min
                           for joint in self.commander.hand.allJoints}

        # starting position for the hand
        self.start_pos = {"THJ1": 0, "THJ2": 0, "THJ3": 0, "THJ4": 0, "THJ5": 0,
                          "FFJ0": 0, "FFJ3": 0, "FFJ4": 0,
                          "MFJ0": 0, "MFJ3": 0, "MFJ4": 0,
                          "RFJ0": 0, "RFJ3": 0, "RFJ4": 0,
                          "LFJ0": 0, "LFJ3": 0, "LFJ4": 0, "LFJ5": 0,
                          "WRJ1": 0, "WRJ2": 0}

    def sinewave_motion(self, joint_name, frequency=0.5, motion_ratio=1.0):
        """
        Repeatedly move a joint following a sinewave function
        @param joint_name - name of the joint to move
        @param frequency - cycles per second
        @param motion_ratio - [0, 1] ratio of joint's full range to perform
        """

        if not self.jnt_coeffs.has_key(joint_name):
            print("invalid joint_name : " + joint_name)
            return

        if motion_ratio > 1.0 or motion_ratio < 0.0:
            print("motion_ratio out of range")
            return

        command = dict(self.start_pos)

        fcoeff = (2.0*pi)/(1.0/frequency)
        amplit = motion_ratio*self.jnt_coeffs[joint_name]
        offset = amplit + self.jnt_offset[joint_name]

        rate = Rate(frequency)
        while self.motion_allowed and not is_shutdown():
            command[joint_name] = amplit*sin(fcoeff*Time.now()) + offset
            self.commander.move_hand(command)
            rate.sleep()

    def channel_joint_mapping(self, channel_to_joint, reader):
        """
        move joints according to map from channels to joints
        """
        for joint_name in channel_to_joint.iter_values():
            if not self.jnt_coeffs.has_key(joint_name):
                print("invalid joint_name : " + joint_name)
                return

        for channel in channel_to_joint.iter_keys():
            if not self.channel_map.has_key(channel):
                print("channel {} is out of range".format(channel))
                return

        if reader is not AnalogReader:
            print("an instance of AnalogReader class must be passed as argument")

        command = dict(self.start_pos)

        rate = Rate(100)
        while self.motion_allowed and not is_shutdown():
            for c, j in channel_to_joint:
                command[j] = self.jnt_coeffs[j]*reader.analog_channel(c) + self.jnt_offset[j]
            self.commander.move_hand(command)
            rate.sleep()