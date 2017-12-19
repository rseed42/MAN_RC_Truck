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
        self.signal = 1400
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


class HumanController:
    def __init__(self, config):
        self.cfg = config
        #
        self.channels = {
            'steering': SteeringChannel(self.cfg.control.steering.default, K_LEFT, K_RIGHT),
            'throttle': ThrottleChannel(self.cfg.control.throttle.default, K_UP, K_DOWN),
            'shift': ControlChannel(self.cfg.control.shift.default, K_w, K_s),
            'leg': ControlChannel(self.cfg.control.leg.default, K_a, K_d)
        }

    def register(self):
        self.channels['steering'].register_publisher(self.cfg.topic.steering, self.cfg.control.msg_queue_size)
        self.channels['throttle'].register_publisher(self.cfg.topic.throttle, self.cfg.control.msg_queue_size)
        self.channels['shift'].register_publisher(self.cfg.topic.shift, self.cfg.control.msg_queue_size)
        self.channels['leg'].register_publisher(self.cfg.topic.leg, self.cfg.control.msg_queue_size)

    def on_key_up(self, event):
        for channel in self.channels.values():
            if event.key == channel.key_increase:
                channel.on_key_increase_up()
            elif event.key == channel.key_decrease:
                channel.on_key_decrease_up()

    def on_key_down(self, event):
        for channel in self.channels.values():
            if event.key == channel.key_increase:
                channel.on_key_increase_down()
            elif event.key == channel.key_decrease:
                channel.on_key_decrease_down()

    def on_key_pressed(self, keys):
        for channel in self.channels.values():
            if keys[channel.key_increase]:
                channel.on_key_increase_pressed()
            elif keys[channel.key_decrease]:
                channel.on_key_decrease_pressed()

