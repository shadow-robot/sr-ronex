from numpy import binary_repr
from rospy import ServiceProxy
from sr_ronex_msgs.srv import SPI

class AnalogReader():
    """
    retrieves values from analog inputs of spi module
    """
    def __init__(self):
        # serials of ronex devices
        serials = ("1402573926", "1402574043")

        # passthrough controller names for each ronex device
        devices = ("/ronex/spi/" + serials[0] + "/command/passthrough/",
                   "/ronex/spi/" + serials[1] + "/command/passthrough/")

        # list of services for each SPI board
        # ordered by ADC index
        self.proxies = [ServiceProxy(devices[0] + '0', SPI),
                        ServiceProxy(devices[0] + '1', SPI),
                        ServiceProxy(devices[1] + '0', SPI),
                        ServiceProxy(devices[1] + '1', SPI),
                        ServiceProxy(devices[1] + '2', SPI),
                        ServiceProxy(devices[1] + '3', SPI)]

        # analog channels per ADC (1,2,3,4,5,6)
        self.channels = ((0,  8, 16, 24),
                         (1,  9, 17, 25),
                         (2, 10, 18, 26),
                         (3, 11, 19, 27),
                         (4, 12, 20, 28),
                         (5, 13, 21, 29))

        # map analog channel to ADC
        self.channel_map = { 0: 0,  1: 1,  2: 2,  3: 3,  4: 4,  5: 5,
                             8: 0,  9: 1, 10: 2, 11: 3, 12: 4, 13: 5,
                            16: 0, 17: 1, 18: 2, 19: 3, 20: 4, 21: 5,
                            24: 0, 25: 1, 26: 2, 27: 3, 28: 4, 29: 5}

        # request for ros services to get analog 0,1,2,3 of each adc
        self.analog_requests = ([6, 0, 0], [6, 64, 0], [6, 128, 0], [6, 192, 0])

        # map analog channel to request index
        self.request_map = { 0: 0,  1: 0,  2: 0,  3: 0,  4: 0,  5: 0,
                             8: 1,  9: 1, 10: 1, 11: 1, 12: 1, 13: 1,
                            16: 2, 17: 2, 18: 2, 19: 2, 20: 2, 21: 2,
                            24: 3, 25: 3, 26: 3, 27: 3, 28: 3, 29: 3}
        # limits of raw analog values
        self.min_raw = 600.0
        self.max_raw = 3300.0

        # coefficient to normalize raw analog values [-1.0, 1.0]
        self.raw_coefficient = 0.0006666666666666666

    def analog_by_channel(self, channel):
        response = self.proxies[self.channel_map[channel]](self.request_map[channel]).data
        binary = "{0}{1}".format(binary_repr(response[1], width=8), binary_repr(response[2], width=8))
        raw_value = int(binary[4:], 2)
        return self.raw_coefficient*(min(max(raw_value, self.min_raw), self.max_raw) - self.min_raw) - 1.0

    def analogs_by_adc(self, adc):
        return [self.analog_by_channel(ch) for ch in self.channels[adc-1]]

    def all_analogs(self):
        return [self.analog_by_channel(ch) for adc in self.channels for ch in adc]
