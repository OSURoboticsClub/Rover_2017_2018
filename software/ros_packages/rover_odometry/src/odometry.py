#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
import serial
from time import time, sleep
import json

from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu

#####################################
# Global Variables
#####################################
NODE_NAME = "tower_odometry"

DEFAULT_PORT = "/dev/rover/ttyOdometry"
# DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200


# DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_GPS_SENTENCE_TOPIC = "gps/sentence"
DEFAULT_IMU_TOPIC = "imu/data"

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
        self.imu_data_topic = rospy.get_param("~imu_data_topic", DEFAULT_IMU_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.odom_serial = serial.Serial(port=self.port, baudrate=self.baud)
        self.odom_serial.setRTS(0)

        self.sentence_publisher = rospy.Publisher(self.gps_sentence_topic, Sentence, queue_size=1)
        self.imu_data_publisher = rospy.Publisher(self.imu_data_topic, Imu, queue_size=1)

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
            line = self.odom_serial.readline()

            temp = json.loads(line)

            gps = temp.get('gps', None)
            imu = temp.get('imu', None)

            if gps:
                self.broadcast_gps(gps)

            if imu:
                self.broadcast_imu(imu)

    def broadcast_gps(self, gps):
        message = Sentence()
        message.header.frame_id = "gps"
        message.header.stamp = rospy.get_rostime()
        message.sentence = gps
        self.sentence_publisher.publish(message)

    def broadcast_imu(self, imu):
        message = Imu()
        message.header.frame_id = "imu"
        message.header.stamp = rospy.get_rostime()

        message.orientation.x = imu["ox"]
        message.orientation.y = imu["oy"]
        message.orientation.z = imu["oz"]
        message.orientation.w = imu["ow"]

        message.angular_velocity.x = imu["avx"]
        message.angular_velocity.y = imu["avy"]
        message.angular_velocity.z = imu["avz"]

        message.linear_acceleration.x = imu["lax"]
        message.linear_acceleration.y = imu["lay"]
        message.linear_acceleration.z = imu["laz"]

        self.imu_data_publisher.publish(message)

if __name__ == "__main__":
    Odometry()
