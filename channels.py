import rospy
from std_msgs.msg import UInt32
from pygame.locals import *
# ------------------------------------------------------------------------------
# Controller
# ------------------------------------------------------------------------------


class ControlChannel(object):
    """
    Base class for control signals. By default nothing changes. Subclass to override channel's behaviour
    """
    def __init__(self, default, key_increase, key_decrease):
        # Used to filter out the signals
        self.key_increase = key_increase
        self.key_decrease = key_decrease
        self.default = default
        self.last_signal = default
        self.signal = default
        self._pub = None

    def register_publisher(self, topic, queue_size):
        self._pub = rospy.Publisher(topic, UInt32, queue_size=queue_size)

    def _send(self):
        self._pub.publish(self.signal)

    def _send_default(self):
        self.last_signal = self.signal
        self.signal = self.default
        self._send()

    def on_key_increase_up(self):
        self._send_default()

    def on_key_decrease_up(self):
        self._send_default()

    def on_key_increase_down(self):
        self._send_default()

    def on_key_decrease_down(self):
        self._send_default()

    def on_key_increase_pressed(self):
        self._send_default()

    def on_key_decrease_pressed(self):
        self._send_default()


class ThrottleChannel(ControlChannel):
    """
    Using just one speed with the throttle, as slow as possible
    """
    def __init__(self, default, key_increase, key_decrease):
        super(ThrottleChannel, self).__init__(default, key_increase, key_decrease)

    def on_key_increase_pressed(self):
        self.last_signal = self.signal
        self.signal = 1350
        self._send()

    def on_key_decrease_pressed(self):
        self.last_signal = self.signal
        self.signal = 1700
        #print("last signal:", self.last_signal)
        #print("current signal:", self.signal)
        #self.last_signal = self.signal
        self._send()

    # no throttle
    def on_key_none(self):
        self.last_signal = self.signal
        self.signal = self.default
        self._send()


class SteeringChannel(ControlChannel):
    """
    Steering needs to be fast, so we send a more powerful signal
    """
    def __init__(self, default, key_increase, key_decrease):
        super(SteeringChannel, self).__init__(default, key_increase, key_decrease)

    # left
    def on_key_increase_pressed(self):
        self.last_signal = self.signal
        self.signal = 1000
        self._send()

    # right
    def on_key_decrease_pressed(self):
        self.last_signal = self.signal
        self.signal = 1800
        self._send()
    
    # no turn
    def on_key_none(self):
        self.last_signal = self.signal
        self.signal = self.default
        self._send()

    # adaptive
    def on_adaptive(self, location=0.5):
        self.last_signal = self.signal
        K_FACTOR = 2
        adaptive_signal = int(1400 - 800 * K_FACTOR * (location - 0.5))
        #print("[INFO]: Steering input value ", adaptive_signal)
        self.signal = adaptive_signal
        self._send()

    def on_key_increase_down(self):
        self.signal = self.last_signal + 500
        self.last_signal = self.signal
        self._send()

    def on_key_decrease_down(self):
        self.signal = self.last_signal - 500
        self.last_signal = self.signal
        self._send()


class ShiftChannel(ControlChannel):
    """
    Using just one speed with the throttle, as slow as possible
    """
    def __init__(self, default, key_increase, key_decrease):
        super(ShiftChannel, self).__init__(default, key_increase, key_decrease)

    def on_key_increase_pressed(self):
        self.last_signal = self.signal
        self._send()

    def on_key_decrease_pressed(self):
        self.last_signal = self.signal
        self._send()

