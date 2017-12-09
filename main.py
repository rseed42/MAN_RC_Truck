#!/usr/bin/python
import rospy
import sys
import pygame
import yaml
import attrdict
# application components
import application
import imagesensor
import distancesensor
import controller
# ------------------------------------------------------------------------------
# Configuration
# ------------------------------------------------------------------------------
CONFIG_FILE_NAME = 'config.yml'
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
    # Image sensor
    image_sensor = imagesensor.ImageSensor(configuration)
    image_sensor.register(configuration.topic.camera.compressed)

    # Forward distance sensor
    forward_distance_sensor = distancesensor.DistanceSensor(configuration.sensor.distance.forward)
    forward_distance_sensor.register(configuration.sensor.distance.forward.topic)

    # Backward distance sensor
    backward_distance_sensor = distancesensor.DistanceSensor(configuration.sensor.distance.backward)
    backward_distance_sensor.register(configuration.sensor.distance.backward.topic)

    # Create and register the controller
    controller = controller.HumanController(configuration)
    controller.register()

    # Initialize this application as an ROS node, otherwise we can not communicate with the ROS system
    rospy.init_node(configuration.ros.node.name, anonymous=True)

    # Instantiate the pygame app
    app = application.App(configuration, image_sensor, controller, forward_distance_sensor, backward_distance_sensor)
    app.on_execute()
