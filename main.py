#!/usr/bin/python
import rospy
from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage
import time
import sys
import pygame
from pygame.locals import *
import pygame.surfarray as surfarray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import yaml
import attrdict

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------
CONFIG_FILE_NAME = 'config.yml'
# ------------------------------------------------------------------------------
# Camera Sensor
# ------------------------------------------------------------------------------
class ImageSensor:
    def __init__(self, configuration):
        # Over how many frames back to calculate the moving average
        self._framerate_avg_count = configuration.sensor.image.framerate_avg_count
        self.average_framerate = 0
        # Used to measure frame length duration
        self.last_time = 0
        self.image_counter = 0L
        self.bridge = CvBridge()
        self.last_image = np.zeros((configuration.sensor.image.width, configuration.sensor.image.height, 3), dtype=np.uint8)
        self._sub = None

    def register(self, topic):
        self._sub = rospy.Subscriber(topic, CompressedImage, self.on_image_update)
        self.last_time = time.time()

    def on_image_update(self, message_data):
        """
        Fetch the image from the topic and calculate the average frame rate
        :param message_data:
        :return:
        """
        # Get the image
        try:
            # The image should be already encoded as rgb8, we pass through to avoid costly recomputing
            image_array = self.bridge.compressed_imgmsg_to_cv2(message_data, desired_encoding="passthrough")
            # For some reason the cv2 transformation rotates the image, haven't figured out why yet
            self.last_image = cv2.rotate(image_array, cv2.ROTATE_90_COUNTERCLOCKWISE)
        except CvBridgeError as err:
            print err

        # Calculate the frame rate
        self.image_counter += 1
        now = time.time()
        frame_duration = now - self.last_time
        framerate = 1./frame_duration
        # Calculate the average framerate from the latest update
        self.average_framerate = self.average_framerate + float(framerate - self.average_framerate)/(self.image_counter + 1)
        # End of this frame
        self.last_time = now

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
        self.signal = 1400
        self._send()

    def on_key_decrease_pressed(self):
        self.last_signal = self.signal
        self.signal = 1600
        self._send()


class SteeringChannel(ControlChannel):
    """
    Steering needs to be fast, so we send a more powerful signal
    """
    def __init__(self, default, key_increase, key_decrease):
        super(SteeringChannel, self).__init__(default, key_increase, key_decrease)

    def on_key_increase_pressed(self):
        self.last_signal = self.signal
        self.signal = 1200
        self._send()

    def on_key_decrease_pressed(self):
        self.last_signal = self.signal
        self.signal = 1800
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

# ------------------------------------------------------------------------------
# App
# ------------------------------------------------------------------------------
class App:
    def __init__(self, configuration, image_sensor, controller):
        self.cfg = configuration
        self._running = True
        self._display_surf = None
        self.font = pygame.font.SysFont(self.cfg.app.surface.font, self.cfg.app.surface.font_size)
        self.clock = pygame.time.Clock()
        # Sensors
        self.image_sensor = image_sensor
        self.controller = controller

    def on_init(self):
        # Initialize the primary display surface
        self._display_surf = pygame.display.set_mode(
            (self.cfg.app.surface.width, self.cfg.app.surface.height),
            pygame.HWSURFACE | pygame.DOUBLEBUF
        )
        self._running = True
        # Return true if everything went well, otherwise we have to abort the application
        return True

    def show_text(self, msg, x, y):
        text = self.font.render(msg, True, self.cfg.text.default.color, self.cfg.text.default.background_color)
        self._display_surf.blit(text, (x, y))

    def on_event(self, event):
        if event.type == pygame.KEYUP:
            self.controller.on_key_up(event)

        if event.type == pygame.KEYDOWN:
            if event.key == K_q:
                self._running = False
            self.controller.on_key_down(event)

        if event.type == pygame.QUIT:
            self._running = False


    def on_key_pressed(self, keys):
        self.controller.on_key_pressed(keys)

    def on_loop(self):
        pass

    def on_render(self):
        self._display_surf.fill(self.cfg.app.surface.screen_color)

        # Render the image and all information on this frame
        surfarray.blit_array(self._display_surf, self.image_sensor.last_image)

        self.show_text('image fps: %.1f' % self.image_sensor.average_framerate, 0, 0)

        # Show all control values
        def show_signal((index, (name, channel))):
            self.show_text('{}: {}'.format(name, channel.signal), 0, 30 + 15*index)
        map(show_signal, enumerate(self.controller.channels.items()))

        # Update the surface after we have applied all drawing operations
        pygame.display.flip()
        # Fixed fps rate for the application
        self.clock.tick(self.cfg.app.clock_rate)

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        # If we can not initialize the application, quit
        if not self.on_init():
            self._running = False
        while self._running:
            self.on_key_pressed(pygame.key.get_pressed())
            for event in pygame.event.get():
                self.on_event(event)
            self.on_loop()
            self.on_render()
        self.on_cleanup()

# ------------------------------------------------------------------------------
# Main
# ------------------------------------------------------------------------------
if __name__ == '__main__':
    try:
        with open(CONFIG_FILE_NAME, 'r') as fp:
            configuration = attrdict.AttrDict(yaml.load(fp))
    except yaml.YAMLError as err:
        sys.stderr.write('Error parsing config file {}:\n'.format(CONFIG_FILE_NAME))
        sys.stderr.write(str(err) + '\n')
        sys.exit(1)
    except IOError as err:
        sys.stderr.write('Could not find the configuration file "{}"\n'.format(CONFIG_FILE_NAME))
        sys.exit(1)

    pygame.init()

    # Create and register sensors. They start receiving messages as soon as they are registered
    image_sensor = ImageSensor(configuration)
    image_sensor.register(configuration.topic.camera.compressed)

    # Create and register the controller
    controller = HumanController(configuration)
    controller.register()

    # Initialize this application as an ROS node, otherwise we can not communicate with the ROS system
    rospy.init_node(configuration.ros.node.name, anonymous=True)

    # Instantiate the pygame app
    app = App(configuration, image_sensor, controller)
    app.on_execute()
