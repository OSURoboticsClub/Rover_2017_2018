#!/usr/bin/env python
import rospy
import time

from rover_control.msg import MiningControlMessage

DEFAULT_TOWER_PAN_TILT_CONTROL_TOPIC = "/rover_control/mining/control"

rospy.init_node("effectors_tester")

publisher = rospy.Publisher(DEFAULT_TOWER_PAN_TILT_CONTROL_TOPIC, MiningControlMessage, queue_size=1)

time.sleep(2)

message = MiningControlMessage()
message.lift_set = 200
message.tilt_set = 1023
message.cal_factor = -1

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
