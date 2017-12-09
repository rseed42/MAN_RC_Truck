import time
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
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
        # Calculate the average frame rate from the latest update
        self.average_framerate = self.average_framerate + float(framerate - self.average_framerate)/(self.image_counter + 1)
        # End of this frame
        self.last_time = now

