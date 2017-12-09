#!/usr/bin/python
import rospy
from std_msgs.msg import UInt32
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
import random
import time
import os
import sys
import pygame
from pygame.locals import *
import pygame.surfarray as surfarray
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------
MSG_QUEUE_SIZE = 10
TOPIC_THROTTLE = '/controller/throttle'
TOPIC_STEERING = '/controller/steering'
TOPIC_SHIFT = '/controller/shift'
TOPIC_LEG = '/controller/leg'
TOPIC_IMAGE = '/raspicam_node/image/compressed'

IMAGE_WIDTH = 308
IMAGE_HEIGHT = 410

CLOCK_RATE = 60
THROTTLE_DEFAULT = 1500
STEERING_DEFAULT = 1500
SHIFT_DEFAULT = 1500
LEG_DEFAULT = 1500
THROTTLE_INCREMENT = 5
STEERING_INCREMENT = 5
SHIFT_INCREMENT = 5
LEG_INCREMENT = 5
# ------------------------------------------------------------------------------
# Init
# ------------------------------------------------------------------------------
def init_servo():
    pub = rospy.Publisher(TOPIC_THROTTLE, UInt32, queue_size=MSG_QUEUE_SIZE)
    rospy.init_node('publisher', anonymous=True)
    pub.publish(1500)
# ------------------------------------------------------------------------------
# Dispatch
# ------------------------------------------------------------------------------
def dispatch(key):
    global running
    if key == u'\x1bOA':
        print 'forward'
    elif key == u'\x1bOB':
        print 'backward'
    elif key == u'\x1bOD':
        print 'left'
    elif key == u'\x1bOC':
        print 'right'
    elif key == u'q':
        running = False
# ------------------------------------------------------------------------------
# Producer
# ------------------------------------------------------------------------------
#    try:
#        producer()
#    except rospy.ROSInterruptException:
#        pass
def producer():
    pub = rospy.Publisher(TOPIC_THROTTLE, UInt32, queue_size=MSG_QUEUE_SIZE)
    rospy.init_node('publisher', anonymous=True)
    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        msg = random.randint(1000, 2000)
        print "message: %d" % msg
        pub.publish(msg)
        rate.sleep()
        rate = rospy.Rate(random.uniform(0.1, 1))
# ------------------------------------------------------------------------------
# App
# ------------------------------------------------------------------------------
class App:
    def __init__(self, width, height):
        self._running = True
        self._display_surf = None
        self.width = width
        self.height = height
        self.size = self.width, self.height
        self.font = pygame.font.SysFont('monospace', 15)
        self.text_color = (255, 255, 255)
        self.screen_color = (0, 0, 0)
        self.clock = pygame.time.Clock()
        self.throttle = THROTTLE_DEFAULT
        self.steering = STEERING_DEFAULT
        self.shift = SHIFT_DEFAULT
        self.leg = LEG_DEFAULT
        self.throttle_pub = rospy.Publisher(TOPIC_THROTTLE, UInt32, queue_size=MSG_QUEUE_SIZE)
        self.steering_pub = rospy.Publisher(TOPIC_STEERING, UInt32, queue_size=MSG_QUEUE_SIZE)
        self.shift_pub = rospy.Publisher(TOPIC_SHIFT, UInt32, queue_size=MSG_QUEUE_SIZE)
        self.leg_pub = rospy.Publisher(TOPIC_LEG, UInt32, queue_size=MSG_QUEUE_SIZE)

        self.image_sub = rospy.Subscriber(TOPIC_IMAGE, CompressedImage, self.image_callback)

        self.framerate_smooth = 0
        self.last_time = time.time()
        self.image_counter = 0
        self.bridge = CvBridge()
        self.last_image = np.zeros((IMAGE_WIDTH, IMAGE_HEIGHT, 3))

    def image_callback(self, image):
        # Calculate the frame rate
        self.image_counter += 1
        now = time.time()
        delta = now - self.last_time
        framerate = 1./delta
        self.framerate_smooth = self.framerate_smooth + float(framerate - self.framerate_smooth)/(self.image_counter+1)
        self.last_time = now

        if self.image_counter % 30 == 0:
            print 'Image Framerate: {}'.format(self.framerate_smooth)

        # Get the image
        try:
            self.last_image = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as err:
            print err


    def on_init(self):
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._running = True
        rospy.init_node('publisher', anonymous=True)

    def show_text(self, msg, left, right):
        text = self.font.render(msg, True, self.text_color, (75, 75, 75))
        self._display_surf.blit(text, (left, right))

    def on_event(self, event):
        if event.type == pygame.KEYUP:
            if event.key == K_UP or event.key == K_DOWN:
                self.throttle = THROTTLE_DEFAULT
                self.throttle_pub.publish(self.throttle)

            if event.key == K_LEFT or event.key == K_RIGHT:
                self.steering = STEERING_DEFAULT
                self.steering_pub.publish(self.steering)

            if event.key == K_w or event.key == K_s:
                self.shift = SHIFT_DEFAULT
                self.shift_pub.publish(self.shift)

            if event.key == K_a or event.key == K_d:
                self.leg = LEG_DEFAULT
                self.leg_pub.publish(self.leg)

            if event.key == K_q:
                self._running = False

        if event.type == pygame.QUIT:
            self._running = False

    def on_key_pressed(self, keys):
        if keys[pygame.K_UP]:
            self.throttle = max(self.throttle - THROTTLE_INCREMENT, 1000)
            self.throttle_pub.publish(self.throttle)

        if keys[pygame.K_DOWN]:
            self.throttle = min(self.throttle + THROTTLE_INCREMENT, 2000)
            self.throttle_pub.publish(self.throttle)

        if keys[pygame.K_LEFT]:
            self.steering = max(self.steering - STEERING_INCREMENT, 1000)
            self.steering_pub.publish(self.steering)

        if keys[pygame.K_RIGHT]:
            self.steering = min(self.steering + STEERING_INCREMENT, 2000)
            self.steering_pub.publish(self.steering)

        if keys[pygame.K_w]:
            self.shift = max(self.shift - SHIFT_INCREMENT, 1000)
            self.shift_pub.publish(self.shift)

        if keys[pygame.K_s]:
            self.shift = min(self.shift + SHIFT_INCREMENT, 2000)
            self.shift_pub.publish(self.shift)

        if keys[pygame.K_a]:
            self.leg = max(self.leg - LEG_INCREMENT, 1000)
            self.leg_pub.publish(self.leg)

        if keys[pygame.K_d]:
            self.leg = min(self.leg + LEG_INCREMENT, 2000)
            self.leg_pub.publish(self.leg)

    def on_loop(self):
        pass

    def on_render(self):
        self._display_surf.fill(self.screen_color)

        surfarray.blit_array(self._display_surf, self.last_image)

        self.show_text('throttle: %d' % self.throttle, 10, 10)
        self.show_text('steering: %d' % self.steering, 10, 30)
        self.show_text('shift: %d' % self.shift, 10, 50)
        self.show_text('leg: %d' % self.leg, 10, 70)

        pygame.display.flip()

        self.clock.tick(CLOCK_RATE)

    def on_cleanup(self):
        pygame.quit()

    def on_execute(self):
        if self.on_init() == False:
          self._running = False

        while(self._running):
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
    pygame.init()
    app = App(IMAGE_WIDTH, IMAGE_HEIGHT)
    app.on_execute()
