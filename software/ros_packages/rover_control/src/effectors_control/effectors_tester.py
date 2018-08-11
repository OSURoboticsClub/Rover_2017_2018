#!/usr/bin/env python
import rospy
import time

from rover_control.msg import CameraControlMessage

TOPIC = "/rover_control/camera/control"

rospy.init_node("effectors_tester")

publisher = rospy.Publisher(TOPIC, CameraControlMessage, queue_size=1)

time.sleep(2)

message = CameraControlMessage()
message.camera_mode = 2

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
