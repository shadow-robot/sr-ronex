from rospy import ServiceProxy, is_shutdown, init_node
from sr_ronex_msgs.srv import SPI
#from std_msgs.msg import Float64
from sr_hand.shadowhand_commander import Commander

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

        # list of services for each SPI board
        # ordered by ADC index
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

        # all valid channels
        self.valid_channels = tuple(self.channel_map.iterkeys())

        # limits of raw analog values
        self.min = 600.0
        self.max = 3300.0

        # coefficient to normalize raw analog values [600, 3300] to range [-1.0, 1.0]
        self.raw_coefficient = (1.0 - (-1.0)) / (self.max - self.min)

    def analog_by_channel(self, channel):
        """
        get the current analog reading of given channel
        self.channels contains valid values for channel
        """
        if channel not in self.valid_channels:
            print("channel {} is out of range".format(channel))
            print("valid channels = {}".format(self.valid_channels))
            return

        request = self.analog_requests[self.request_map[channel]]
        response = self.proxies[self.channel_map[channel]](request).data

        # union of the high niblle of response[1] (4 bits) and response[2]
        raw_value = 0x100*(ord(response[1]) & 0x0F) + ord(response[2])
        return self.raw_coefficient*(min(max(raw_value, self.min), self.max) - self.min) - 1.0

    def analogs_by_adc(self, adc):
        """
        get list of current analog readings for all channels of ADC
        basically for each row of self.channels
        valid adc values are [1, 6]
        """
        if adc not in xrange(1, 7):
            print("adc {0} out of range {1}".format(adc, tuple(xrange(1, 7))))
            return
        return [self.analog_by_channel(ch) for ch in self.channels[adc - 1]]

    def all_analogs(self):
        """
        get list of all available analog readings
        """
        return [self.analog_by_channel(ch) for adc in self.channels for ch in adc]


class JointMotions(object):
    """
    Helps to execute some joint motions
    """
    def __init__(self):
        joints = ("lfj0", "lfj3", "lfj4", "lfj5",
                  "rfj0", "rfj3", "rfj4",
                  "mfj0", "mfj3", "mfj4",
                  "ffj0", "ffj3", "ffj4",
                  "thj1", "thj2", "thj3", "thj4", "thj5")

        #     joint max degs  0 90  3 45  4 -20  5 20   finger
        limits =             (1.57, 0.79, -0.30, 0.30,  # lf
                              1.57, 0.79, -0.30,        # rf
                              1.57, 0.79, -0.30,        # mf
                              1.57, 0.79, -0.30,        # ff
        #            thumb    1 45  2 20  3  5  4 70  5 30
                              0.79, 0.30, 0.09, 1.22, 0.52)

        self.joint_coeffs = {jn : (lim/1.57) for jn, lim in zip(joints, limits)}

        self.motion_allowed = True

        init_node("AkronJointMotions")

        self.commander = Commander()


    def sinewave_motion(self, joint_name):
        """
        Repeatedly move a joint following a sinewave function
        """
        while self.motion_allowed and not is_shutdown():
            pass

    def channel_joint_mapping(self, channel, joint):
        """
        Map readings of analog channel joint motion
        """
        while self.motion_allowed and not is_shutdown():
            pass