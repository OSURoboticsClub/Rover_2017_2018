#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep
import json

import serial.rs485

from nmea_msgs.msg import Sentence


#####################################
# Global Variables
#####################################
NODE_NAME = "tower_odometry"

# DEFAULT_PORT = "/dev/rover/ttyOdometry"
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


# DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_GPS_SENTENCE_TOPIC = "/nmea_sentence"
DEFAULT_DRIVE_CONTROL_STATUS_TOPIC = "drive_status/rear"

DEFAULT_HERTZ = 100


#####################################
# DriveControl Class Definition
#####################################
class Odometry(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.gps_sentence_topic = rospy.get_param("~gps_sentence_topic", DEFAULT_GPS_SENTENCE_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.odom_serial = serial.Serial(port=self.port, baudrate=self.baud)
        self.odom_serial.setRTS(0)

        self.sentence_publisher = rospy.Publisher(self.gps_sentence_topic, Sentence, queue_size=1)

        self.run()


    def run(self):
        while not rospy.is_shutdown():
            start_time = time()

            try:
                self.process_messages()

            except Exception, error:
                print "Error occurred:", error


            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))

    def process_messages(self):
        if self.odom_serial.inWaiting():
            line = self.odom_serial.readline()  # type:s
            # print line

            temp = json.loads(line)
            gps = temp.get('gps', None)

            if gps:
                self.broadcast_gps(gps)

            imu = temp.get('imu', None)

            if imu:
                print imu


            # if not line.startswith("ox"):
            #     print line,
            # if line[0] == '$':
            #     print line,

    def broadcast_gps(self, gps):
        # print gps
        self.sentence_publisher.publish(Sentence(sentence=gps))


if __name__ == "__main__":
    Odometry()