#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep

# Custom Imports
from rover_control.msg import DriveCommandMessage, DriveControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "control_coordinator"

DEFAULT_IRIS_DRIVE_COMMAND_TOPIC = "command_control/iris_drive"
DEFAULT_REAR_BOGIE_TOPIC = "drive_control/rear"
DEFAULT_LEFT_BOGIE_TOPIC = "drive_control/left"
DEFAULT_RIGHT_BOGIE_TOPIC = "drive_control/right"

UINT16_MAX = 65535


#####################################
# ControlCoordinator Class Definition
#####################################
class ControlCoordinator(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.iris_topic = rospy.get_param("~iris_drive_command_topic", DEFAULT_IRIS_DRIVE_COMMAND_TOPIC)
        self.rear_bogie_topic = rospy.get_param("~rear_bogie_control_topic", DEFAULT_REAR_BOGIE_TOPIC)
        self.left_bogie_topic = rospy.get_param("~left_bogie_control_topic", DEFAULT_LEFT_BOGIE_TOPIC)
        self.right_bogie_topic = rospy.get_param("~right_bogie_control_topic", DEFAULT_RIGHT_BOGIE_TOPIC)

        self.iris_drive_command_subscriber = \
            rospy.Subscriber(self.iris_topic, DriveCommandMessage, self.iris_drive_command_callback)

        self.rear_bogie_publisher = rospy.Publisher(self.rear_bogie_topic, DriveControlMessage, queue_size=1)
        self.left_bogie_publisher = rospy.Publisher(self.left_bogie_topic, DriveControlMessage, queue_size=1)
        self.right_bogie_publisher = rospy.Publisher(self.right_bogie_topic, DriveControlMessage, queue_size=1)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            sleep(0.25)

    def iris_drive_command_callback(self, drive_command):
        rear_drive = DriveControlMessage()
        left_drive = DriveControlMessage()
        right_drive = DriveControlMessage()

        left = drive_command.drive_twist.linear.x - drive_command.drive_twist.angular.z
        right = drive_command.drive_twist.linear.x + drive_command.drive_twist.angular.z

        left_direction = 1 if left >= 0 else 0
        rear_drive.first_motor_direction = left_direction
        left_drive.first_motor_direction = left_direction
        left_drive.second_motor_direction = left_direction

        right_direction = 1 if right >= 0 else 0
        rear_drive.second_motor_direction = right_direction
        right_drive.first_motor_direction = right_direction
        right_drive.second_motor_direction = right_direction

        left_speed = abs(left * UINT16_MAX)
        right_speed = abs(right * UINT16_MAX)

        rear_drive.first_motor_speed = left_speed
        left_drive.first_motor_speed = left_speed
        left_drive.second_motor_speed = left_speed

        rear_drive.second_motor_speed = right_speed
        right_drive.first_motor_speed = right_speed
        right_drive.second_motor_speed = right_speed

        self.rear_bogie_publisher.publish(rear_drive)
        self.left_bogie_publisher.publish(left_drive)
        self.right_bogie_publisher.publish(right_drive)

if __name__ == "__main__":
    ControlCoordinator()
