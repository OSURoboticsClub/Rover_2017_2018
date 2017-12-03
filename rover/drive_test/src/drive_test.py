#!/usr/bin/env python

import rospy
import math
import tf
from tf.transformations import euler_from_quaternion
import message_filters
from pprint import pprint
import time
# The laser scan message
from sensor_msgs.msg import Joy

# The odometry message
from nav_msgs.msg import Odometry

# the velocity command message
from geometry_msgs.msg import Twist

# global pi - this may come in handy
pi = math.pi

# method to control the robot
def callback_spacenav(twist_message):
    scalar_forward = 100.0
    scalar_turn = 2.0

    # make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = twist_message.linear.x * scalar_forward
    command.linear.y = twist_message.linear.y * scalar_forward
    command.linear.z = twist_message.linear.z * scalar_forward
    command.angular.x = twist_message.angular.x * scalar_forward
    command.angular.y = twist_message.angular.y * scalar_forward
    command.angular.z = twist_message.angular.z * scalar_turn

    # print command

    # Send the commands
    pub.publish(command)

# method to control the robot
def callback_joy(joy_message):
    scalar_forward = 100.0
    scalar_turn = 2.0

    # make a new twist message
    command = Twist()

    # Fill in the fields.  Field values are unspecified 
    # until they are actually assigned. The Twist message 
    # holds linear and angular velocities.
    command.linear.x = joy_message.axes[1] * 100
    #command.linear.y = twist_message.linear.y * scalar_forward
    #command.linear.z = twist_message.linear.z * scalar_forward
    #command.angular.x = twist_message.angular.x * scalar_forward
    #command.angular.y = twist_message.angular.y * scalar_forward
    command.angular.z = joy_message.axes[0] * 1.0

    # command.angular.z = joy_message.axes[2] * 1.0

    print joy_message

    # Send the commands
    pub.publish(command)

# main function call
if __name__ == "__main__":
    # Initialize the node
    rospy.init_node('drive_test', log_level=rospy.DEBUG)

    # subscribe to laser scan message
    # sub = rospy.Subscriber('/spacenav/twist', Twist, callback_spacenav)

    # subscribe to laser scan message
    sub = rospy.Subscriber('/joy', Joy, callback_joy)

    # synchronize laser scan and odometry data
    # ts = message_filters.TimeSynchronizer([sub, sub2], 10)
    # ts.registerCallback(callback)

    # publish twist message
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Turn control over to ROS
    rospy.spin()

