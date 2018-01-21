#!/usr/bin/env python
import rospy
from system_statuses.msg import RoverSysStatus


class RoverStatuses:

    def __init__(self):

        rospy.init_node('RoverStatuses')

        self.pub = rospy.Publisher('rover_statuses_chatter', RoverSysStatus, queue_size=10)

        # Test Subscription for updating camera_undercarriage
        rospy.Subscriber('rover_system_status_chatter', RoverSysStatus, self.__sub_callback)

        self.msg = RoverSysStatus()

    def __sub_callback(self, data):
        self.msg.UTC_GPS_time = data.UTC_GPS_time
        self.msg.bogie_connection_1 = data.bogie_connection_1
        self.msg.bogie_connection_2 = data.bogie_connection_2
        self.msg.bogie_connection_3 = data.bogie_connection_3
        self.msg.arm_connection_status = data.arm_connection_status
        self.msg.arm_end_effector_connection_statuses = data.arm_end_effector_connection_statuses
        self.msg.camera_zed = data.camera_zed
        self.msg.camera_undercarriage = data.camera_undercarriage
        self.msg.camera_chassis = data.camera_chassis
        self.msg.camera_main_navigation = data.camera_main_navigation
        self.msg.sample_containment_connection_status = data.sample_containment_connection_status
        self.msg.tower_connection_status = data.tower_connection_status
        self.msg.chassis_pan_tilt_connection_status = data.chassis_pan_tilt_connection_status
        self.msg.GPS_connection_status = data.GPS_connection_status
        self.msg.jetson_CPU = data.jetson_CPU
        self.msg.jetson_RAM = data.jetson_RAM
        self.msg.jetson_EMMC = data.jetson_EMMC
        self.msg.jetson_NVME_SSD = data.jetson_NVME_SSD
        self.msg.FrSky_controller_connection_status = data.FrSky_controller_connection_status
        rospy.loginfo(self.msg)

    def run(self):
        rospy.Subscriber('rover_system_status_chatter', RoverSysStatus, self.__sub_callback)

        # Update GUI Here? (Most likely)

        rospy.spin()


if __name__ == '__main__':
    rover_statuses = RoverStatuses()
    rover_statuses.run()
