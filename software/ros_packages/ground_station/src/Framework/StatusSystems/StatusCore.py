#!/usr/bin/env python

import rospy
from rover_status.msg import *
from PyQt5 import QtWidgets, QtCore, QtGui, uic


class SensorCore(QtCore.QThread):

    def __init__(self, screen):
        super(SensorCore, self).__init__()

        self.not_abort = True

        self.screen_main_window = screen

        self.cpu_read = self.screen_main_window.lineEdit  # type: QtWidgets.QLabel
        self.ram_read = self.screen_main_window.lineEdit_2  # type: QtWidgets.QLabel

        rospy.init_node('SensorCore')

        # self.pub = rospy.Publisher('rover_statuses_chatter', RoverSysStatus, queue_size=10)

        # Subscription examples on pulling data from system_statuses_node.py
        rospy.Subscriber('camera_system_status_chatter', CameraStatuses, self.__camera_callback)
        rospy.Subscriber('bogie_system_status_chatter', BogieStatuses, self.__bogie_callback)
        rospy.Subscriber('FrSky_system_status_chatter', FrSkyStatus, self.__frsky_callback)
        rospy.Subscriber('GPS_system_status_chatter', GPSInfo, self.__gps_callback)
        rospy.Subscriber('jetson_system_status_chatter', JetsonInfo, self.__jetson_callback)
        rospy.Subscriber('misc_system_status_chatter', MiscStatuses, self.__misc_callback)

        self.camera_msg = CameraStatuses()
        self.bogie_msg = BogieStatuses()
        self.FrSky_msg = FrSkyStatus()
        self.GPS_msg = GPSInfo()
        self.jetson_msg = JetsonInfo()
        self.misc_msg = MiscStatuses()

    def __camera_callback(self, data):
        self.camera_msg.camera_zed = data.camera_zed
        self.camera_msg.camera_undercarriage = data.camera_undercarriage
        self.camera_msg.camera_chassis = data.camera_chassis
        self.camera_msg.camera_main_navigation = data.camera_main_navigation

    def __frsky_callback(self, data):
        self.FrSky_msg.FrSky_controller_connection_status = data.FrSky_controller_connection_status

    def __bogie_callback(self, data):
        self.bogie_msg.bogie_connection_1 = data.bogie_connection_1
        self.bogie_msg.bogie_connection_2 = data.bogie_connection_2
        self.bogie_msg.bogie_connection_3 = data.bogie_connection_3

    def __jetson_callback(self, data):
        self.jetson_msg.jetson_CPU = data.jetson_CPU
        self.cpu_read.setText(str(self.jetson_msg.jetson_CPU))
        self.jetson_msg.jetson_RAM = data.jetson_RAM
        self.ram_read.setText(str(self.jetson_msg.jetson_RAM))
        self.jetson_msg.jetson_EMMC = data.jetson_EMMC
        self.jetson_msg.jetson_NVME_SSD = data.jetson_NVME_SSD
        #rospy.loginfo(self.jetson_msg)

    def __gps_callback(self, data):
        self.GPS_msg.UTC_GPS_time = data.UTC_GPS_time
        self.GPS_msg.GPS_connection_status = data.GPS_connection_status

    def __misc_callback(self, data):
        self.misc_msg.arm_connection_status = data.arm_connection_status
        self.misc_msg.arm_end_effector_connection_statuses = data.arm_end_effector_connection_statuses
        self.misc_msg.sample_containment_connection_status = data.sample_containment_connection_status
        self.misc_msg.tower_connection_status = data.tower_connection_status
        self.misc_msg.chassis_pan_tilt_connection_status = data.chassis_pan_tilt_connection_status

    def run(self):
        rospy.Subscriber('camera_system_status_chatter', CameraStatuses, self.__camera_callback)
        rospy.Subscriber('bogie_system_status_chatter', BogieStatuses, self.__bogie_callback)
        rospy.Subscriber('FrSky_system_status_chatter', FrSkyStatus, self.__frsky_callback)
        rospy.Subscriber('GPS_system_status_chatter', GPSInfo, self.__gps_callback)
        rospy.Subscriber('jetson_system_status_chatter', JetsonInfo, self.__jetson_callback)
        rospy.Subscriber('misc_system_status_chatter', MiscStatuses, self.__misc_callback)
        #self.gui_element = self.jetson_msg.jetson_CPU
        #print(self.jetson_msg.jetson_CPU)
        rospy.spin()


    def on_kill_threads_requested__slot(self):
        self.not_abort = False

    def connect_signals_and_slots(self):
        pass


if __name__ == '__main__':
    rover_statuses = SensorCore()
    rover_statuses.run()