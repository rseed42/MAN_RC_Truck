import time
import rospy
from sensor_msgs.msg import Range
# ------------------------------------------------------------------------------
# Distance Sensor
# ------------------------------------------------------------------------------


class DistanceSensor:
    def __init__(self, configuration):
        # Over how many frames back to calculate the moving average
        # self._framerate_avg_count = configuration.framerate_avg_count
        self._framerate_avg_count = 15
        self.average_framerate = 0
        # Used to measure frame length duration
        self.last_time = 0
        self.counter = 0L
        self.last_value = 0
        self._sub = None
        self.expected_framerate = configuration.expected_distance_framerate

    def register(self, topic):
        self._sub = rospy.Subscriber(topic, Range, self.on_value_update)
        self.last_time = time.time()

    # This could be static, since forward/backward are coupled
    def on_value_update(self, msg):
        """
        :param message_value:
        :return:
        """
        self.last_value = msg.range
        # Calculate the frame rate
        self.counter += 1
        now = time.time()
        frame_duration = now - self.last_time
        framerate = 1./frame_duration
        # There are rare huge spikes for some reason, we need to filter them out from the average
        # framerate since they distort it significantly
        if framerate > 1.25 * self.expected_framerate:
            framerate = self.expected_framerate
        # Calculate the average frame rate from the latest update
        self.average_framerate = self.average_framerate + float(framerate - self.average_framerate)/(self.counter + 1)
        # self.average_framerate = framerate
        # End of this frame
        self.last_time = now

