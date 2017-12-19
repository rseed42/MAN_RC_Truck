from pygame.locals import *
import channels
# ------------------------------------------------------------------------------
# Human Controller
# ------------------------------------------------------------------------------


class HumanController:
    def __init__(self, config):
        self.cfg = config
        #
        self.channels = {
            'steering': channels.SteeringChannel(self.cfg.control.steering.default, K_LEFT, K_RIGHT),
            'throttle': channels.ThrottleChannel(self.cfg.control.throttle.default, K_UP, K_DOWN),
            'shift': channels.ControlChannel(self.cfg.control.shift.default, K_w, K_s),
            'leg': channels.ControlChannel(self.cfg.control.leg.default, K_a, K_d)
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


class HybridController:
    def __init__(self, config):
        self.cfg = config
        #
        self.channels = {
            'steering': channels.SteeringChannel(self.cfg.control.steering.default, K_LEFT, K_RIGHT),
            'throttle': channels.ThrottleChannel(self.cfg.control.throttle.default, K_UP, K_DOWN),
            'shift': channels.ControlChannel(self.cfg.control.shift.default, K_w, K_s),
            'leg': channels.ControlChannel(self.cfg.control.leg.default, K_a, K_d)
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

    # simple with turn signals
    def turn_left(self):
        self.channels['steering'].on_key_increase_pressed()
    
    def turn_right(self):
        self.channels['steering'].on_key_decrease_pressed()

    def straight(self):
        self.channels['steering'].on_key_none()
    
