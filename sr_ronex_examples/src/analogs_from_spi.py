from numpy import binary_repr
from rospy import ServiceProxy
from sr_ronex_msgs.srv import SPI

class AnalogReader():
    """
    retrieves values from analog inputs of spi module
    """
    def __init__(self):
        self.analog_requests = ([6, 0, 0], [6, 64, 0], [6, 128, 0], [6, 192, 0])

        serials = ("1402573926", "1402574043")
        devices = ("/ronex/spi/" + serials[0] + "/command/passthrough/",
                   "/ronex/spi/" + serials[1] + "/command/passthrough/")

        self.proxies = [ServiceProxy(devices[0] + '0', SPI),
                        ServiceProxy(devices[0] + '1', SPI),
                        ServiceProxy(devices[1] + '0', SPI),
                        ServiceProxy(devices[1] + '1', SPI),
                        ServiceProxy(devices[1] + '2', SPI),
                        ServiceProxy(devices[1] + '3', SPI)]

        self.channels = ((0,  8, 16, 24),
                         (1,  9, 17, 25),
                         (2, 10, 18, 26),
                         (3, 11, 19, 27),
                         (4, 12, 20, 28),
                         (5, 13, 21, 29))

        self.channel_map = { 0: 0,  1: 1,  2: 2,  3: 3,  4: 4,  5: 5,
                             8: 0,  9: 1, 10: 2, 11: 3, 12: 4, 13: 5,
                            16: 0, 17: 1, 18: 2, 19: 3, 20: 4, 21: 5,
                            24: 0, 25: 1, 26: 2, 27: 3, 28: 4, 29: 5}

        self.request_map = { 0: [6, 0, 0],  1: [6, 0, 0],  2: [6, 0, 0],  3: [6, 0, 0],  4: [6, 0, 0],  5: [6, 0, 0],
                             8: 0,  9: 1, 10: 2, 11: 3, 12: 4, 13: 5,
                            16: 0, 17: 1, 18: 2, 19: 3, 20: 4, 21: 5,
                            24: 0, 25: 1, 26: 2, 27: 3, 28: 4, 29: 5}

    def analog_by_channel(self, channel):
        proxy = self.proxies[self.channels[channel]]


        return
        responses = (self.proxies[channel](req).data for req in self.analog_requests)
        results = ['{0:08b}{1:08b}'.format(int(res[1]), int(res[2])) for res in responses]

    def analogs_by_module(self, module):


    def all_analogs(self):
