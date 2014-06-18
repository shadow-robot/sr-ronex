import rospy
from sr_ronex_msgs.srv import SPI

class AnalogReader():
    """
    retrieves values from analog inputs of spi module
    """
    def __init__(self):
        self.analog_requests = ([6, 0, 0], [6, 64, 0], [6, 128, 0], [6, 192, 0])
        self.proxies = [rospy.ServiceProxy("/ronex/spi/1385468312/command/passthrough/{}".format(i), SPI) for i in xrange(4)]
        
    def get_emg_values_channel(self, channel):
        responses = (self.proxies[channel](req).data for req in self.analog_requests)
        results = ['{0:08b}{1:08b}'.format(int(res[1]), int(res[2])) for res in responses]

