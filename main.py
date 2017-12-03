#!/usr/bin/python
import rospy
from std_msgs.msg import UInt32
import random
import time
import os
import sys
import pygame
from pygame.locals import *

# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------
MSG_QUEUE_SIZE = 10
TOPIC_THROTTLE = '/controller/throttle'
TOPIC_STEERING = '/controller/steering'
CLOCK_RATE = 60
THROTTLE_DEFAULT = 1500
STEERING_DEFAULT = 1500
THROTTLE_INCREMENT = 5
STEERING_INCREMENT = 5
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
    def __init__(self):
        self._running = True
        self._display_surf = None
        self.size = self.width, self.height = 640, 400
        self.font = pygame.font.SysFont('monospace', 15)
        self.text_color = (255, 255, 255)
        self.screen_color = (0, 0, 0)
        self.clock = pygame.time.Clock()
        self.throttle = THROTTLE_DEFAULT
        self.steering = STEERING_DEFAULT
        self.throttle_pub = rospy.Publisher(TOPIC_THROTTLE, UInt32, queue_size=MSG_QUEUE_SIZE)
        self.steering_pub = rospy.Publisher(TOPIC_STEERING, UInt32, queue_size=MSG_QUEUE_SIZE)

    def on_init(self):
        self._display_surf = pygame.display.set_mode(self.size, pygame.HWSURFACE | pygame.DOUBLEBUF)
        self._running = True
        rospy.init_node('publisher', anonymous=True)

    def show_text(self, msg, left, right):
        text = self.font.render(msg, True, self.text_color)
        self._display_surf.blit(text, (left, right))
        pygame.display.flip()

    def on_event(self, event):
        if event.type == pygame.KEYUP:
            if event.key == K_UP or event.key == K_DOWN:
                self.throttle = THROTTLE_DEFAULT
                self.throttle_pub.publish(self.throttle)

            if event.key == K_LEFT or event.key == K_RIGHT:
                self.steering = STEERING_DEFAULT
                self.steering_pub.publish(self.steering)

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

    def on_loop(self):
        pass

    def on_render(self):
        self._display_surf.fill(self.screen_color)
        self.show_text('throttle: %d' % self.throttle, 10, 10)
        self.show_text('steering: %d' % self.steering, 150, 10)
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
    app = App()
    app.on_execute()
