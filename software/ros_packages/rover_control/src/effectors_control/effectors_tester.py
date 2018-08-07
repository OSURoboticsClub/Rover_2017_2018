#!/usr/bin/env python
import rospy
import time

from rover_control.msg import GripperControlMessage

TOPIC = "/gripper/control"

rospy.init_node("effectors_tester")

publisher = rospy.Publisher(TOPIC, GripperControlMessage, queue_size=1)

time.sleep(2)

message = GripperControlMessage()
message.gripper_mode = 1
message.gripper_position = 0
message.should_home = 0

publisher.publish(message)

time.sleep(5)

# message = MiningControlMessage()
# message.lift_set = -200
# message.tilt_set = -100
# message.cal_factor = -1
#
# publisher.publish(message)
#
# time.sleep(2)
