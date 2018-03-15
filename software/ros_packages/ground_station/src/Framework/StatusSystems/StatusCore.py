#!/usr/bin/env python

import rospy
from rover_status.msg import *
from PyQt5 import QtWidgets, QtCore, QtGui, uic
from std_msgs.msg import Empty

REQUEST_UPDATE_TOPIC = "/rover_status/update_requested"


CAMERA_TOPIC_NAME = "/rover_status/camera_status"
BOGIE_TOPIC_NAME = "/rover_status/bogie_status"
FRSKY_TOPIC_NAME = "/rover_status/frsky_status"
GPS_TOPIC_NAME = "/rover_status/gps_status"
JETSON_TOPIC_NAME = "/rover_status/jetson_status"
MISC_TOPIC_NAME = "/rover_status/misc_status"


class SensorCore(QtCore.QThread):
    # ########## create signals for slots ##########
    jetson_cpu_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_cpu_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_emmc_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_emmc_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_gpu_temp_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_gpu_temp_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    jetson_ram_update_ready__signal = QtCore.pyqtSignal(str)
    jetson_ram_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    bogie_connection_1_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    bogie_connection_2_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    bogie_connection_3_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    camera_zed_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_under_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_chassis_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)
    camera_main_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    gps_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)

    frsky_stylesheet_change_ready__signal = QtCore.pyqtSignal(str)


    def __init__(self, shared_objects):
        super(SensorCore, self).__init__()

        self.run_thread_flag = True

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.screen_main_window = self.shared_objects["screens"]["left_screen"]

        self.cpu_read = self.screen_main_window.lineEdit  # type: QtWidgets.QLabel
        self.ram_read = self.screen_main_window.lineEdit_2  # type: QtWidgets.QLabel

        # ########## set vars to gui elements
        self.rover_conn = self.screen_main_window.rover  # type: QtWidgets.QLabel
        self.frsky = self.screen_main_window.frsky  # type: QtWidgets.QLabel
        self.nav_mouse = self.screen_main_window.nav_mouse  # type: QtWidgets.QLabel
        self.joystick = self.screen_main_window.joystick  # type: QtWidgets.QLabel
        self.gps = self.screen_main_window.gps  # type: QtWidgets.QLabel
        self.zed = self.screen_main_window.zed  # type: QtWidgets.QLabel
        self.main_cam = self.screen_main_window.main_cam  # type: QtWidgets.QLabel
        self.chassis_cam = self.screen_main_window.chassis_cam  # type: QtWidgets.QLabel
        self.under_cam = self.screen_main_window.under_cam  # type: QtWidgets.QLabel
        self.bogie_right = self.screen_main_window.right_bogie  # type: QtWidgets.QLabel
        self.bogie_left = self.screen_main_window.left_bogie  # type: QtWidgets.QLabel
        self.bogie_rear = self.screen_main_window.rear_bogie  # type: QtWidgets.QLabel
        self.clock = self.screen_main_window.clock  # type: QtWidgets.QLCDNumber
        self.cpu = self.screen_main_window.cpu  # type: QtWidgets.QLabel
        self.ram = self.screen_main_window.ram  # type: QtWidgets.QLabel
        self.gpu_temp = self.screen_main_window.gpu_temp  # type: QtWidgets.QLabel
        self.emmc = self.screen_main_window.emmc  # type: QtWidgets.QLabel

        # ########## subscriptions pulling data from system_statuses_node.py ##########
        self.camera_status = rospy.Subscriber(CAMERA_TOPIC_NAME, CameraStatuses, self.__camera_callback)
        self.bogie_status = rospy.Subscriber(BOGIE_TOPIC_NAME, BogieStatuses, self.__bogie_callback)
        self.frsky_status = rospy.Subscriber(FRSKY_TOPIC_NAME, FrSkyStatus, self.__frsky_callback)
        self.gps_status = rospy.Subscriber(GPS_TOPIC_NAME, GPSInfo, self.__gps_callback)
        self.jetson_status = rospy.Subscriber(JETSON_TOPIC_NAME, JetsonInfo, self.__jetson_callback)
        self.misc_status = rospy.Subscriber(MISC_TOPIC_NAME, MiscStatuses, self.__misc_callback)

        self.camera_msg = CameraStatuses()
        self.bogie_msg = BogieStatuses()
        self.FrSky_msg = FrSkyStatus()
        self.GPS_msg = GPSInfo()
        self.jetson_msg = JetsonInfo()
        self.misc_msg = MiscStatuses()

        rospy.Publisher(REQUEST_UPDATE_TOPIC, Empty, queue_size=1).publish(Empty())

    def __camera_callback(self, data):
        self.camera_msg.camera_zed = data.camera_zed
        self.camera_msg.camera_undercarriage = data.camera_undercarriage
        self.camera_msg.camera_chassis = data.camera_chassis
        self.camera_msg.camera_main_navigation = data.camera_main_navigation

        if self.camera_msg.camera_zed is False:
            # self.zed.setStyleSheet("background-color: red;")
            self.camera_zed_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.zed.setStyleSheet("background-color: darkgreen;")
            self.camera_zed_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        if self.camera_msg.camera_undercarriage is False:
            # self.under_cam.setStyleSheet("background-color: red;")
            self.camera_under_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.under_cam.setStyleSheet("background-color: darkgreen;")
            self.camera_under_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        if self.camera_msg.camera_chassis is False:
            # self.chassis_cam.setStyleSheet("background-color: red;")
            self.camera_chassis_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.chassis_cam.setStyleSheet("background-color: darkgreen;")
            self.camera_chassis_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        if self.camera_msg.camera_main_navigation is False:
            # self.main_cam.setStyleSheet("background-color: red;")
            self.camera_main_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.main_cam.setStyleSheet("background-color: darkgreen;")
            self.camera_main_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

    def __frsky_callback(self, data):
        self.FrSky_msg.FrSky_controller_connection_status = data.FrSky_controller_connection_status

        if self.FrSky_msg.FrSky_controller_connection_status is False:
            self.frsky_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            self.frsky_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

    def __bogie_callback(self, data):
        self.bogie_msg.bogie_connection_1 = data.bogie_connection_1
        self.bogie_msg.bogie_connection_2 = data.bogie_connection_2
        self.bogie_msg.bogie_connection_3 = data.bogie_connection_3

        if self.bogie_msg.bogie_connection_1 is False:
            # self.bogie_right.setStyleSheet("background-color: red;")
            self.bogie_connection_1_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.bogie_right.setStyleSheet("background-color: darkgreen;")
            self.bogie_connection_1_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        if self.bogie_msg.bogie_connection_2 is False:
            # self.bogie_left.setStyleSheet("background-color: red;")
            self.bogie_connection_2_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.bogie_left.setStyleSheet("background-color: darkgreen;")
            self.bogie_connection_2_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        if self.bogie_msg.bogie_connection_3 is False:
            # self.bogie_rear.setStyleSheet("background-color: red;")
            self.bogie_connection_3_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.bogie_rear.setStyleSheet("background-color: darkgreen;")
            self.bogie_connection_3_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

    def __jetson_callback(self, data):
        self.jetson_msg.jetson_CPU = data.jetson_CPU

        # self.cpu_read.setText(str(self.jetson_msg.jetson_CPU))
        # self.cpu.setText(str(self.jetson_msg.jetson_CPU))
        self.jetson_cpu_update_ready__signal.emit(str(self.jetson_msg.jetson_CPU))

        if self.jetson_msg.jetson_CPU > 79:
            self.jetson_cpu_stylesheet_change_ready__signal.emit("background-color: orange;")
        elif self.jetson_msg.jetson_CPU > 89:
            self.jetson_cpu_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            self.jetson_cpu_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        self.jetson_ram_update_ready__signal.emit(str(self.jetson_msg.jetson_RAM))

        if self.jetson_msg.jetson_RAM > 79:
            self.jetson_ram_stylesheet_change_ready__signal.emit("background-color: orange;")
        elif self.jetson_msg.jetson_RAM > 89:
            self.jetson_ram_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            self.jetson_ram_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        self.jetson_gpu_temp_update_ready__signal.emit(str(self.jetson_msg.jetson_GPU_temp))

        if self.jetson_msg.jetson_GPU_temp > 64:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit("background-color: orange;")
        elif self.jetson_msg.jetson_GPU_temp > 79:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            self.jetson_gpu_temp_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

        self.jetson_emmc_update_ready__signal.emit(str(self.jetson_msg.jetson_EMMC))

        if self.jetson_msg.jetson_EMMC > 79:
            self.jetson_emmc_stylesheet_change_ready__signal.emit("background-color: orange;")
        elif self.jetson_msg.jetson_EMMC > 89:
            self.jetson_emmc_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            self.jetson_emmc_stylesheet_change_ready__signal.emit("background-color: darkgreen")

    def __gps_callback(self, data):
        self.GPS_msg.UTC_GPS_time = data.UTC_GPS_time
        self.GPS_msg.GPS_connection_status = data.GPS_connection_status

        if self.GPS_msg.GPS_connection_status is False:
            # self.gps.setStyleSheet("background-color: red")
            self.gps_stylesheet_change_ready__signal.emit("background-color: red;")
        else:
            # self.gps.setStyleSheet("background-color: darkgreen;")
            self.gps_stylesheet_change_ready__signal.emit("background-color: darkgreen;")

    def __misc_callback(self, data):
        self.misc_msg.arm_connection_status = data.arm_connection_status
        self.misc_msg.arm_end_effector_connection_statuses = data.arm_end_effector_connection_statuses
        self.misc_msg.sample_containment_connection_status = data.sample_containment_connection_status
        self.misc_msg.tower_connection_status = data.tower_connection_status
        self.misc_msg.chassis_pan_tilt_connection_status = data.chassis_pan_tilt_connection_status

    def __display_time(self):
        time = QtCore.QTime.currentTime()
        temp = time.toString('hh:mm:ss')
        self.clock.display(temp)

    def run(self):
        while self.run_thread_flag:
            self.msleep(100)

    def connect_signals_and_slots(self):
        self.jetson_cpu_update_ready__signal.connect(self.cpu.setText)
        self.jetson_cpu_stylesheet_change_ready__signal.connect(self.cpu.setStyleSheet)
        self.jetson_ram_update_ready__signal.connect(self.ram.setText)
        self.jetson_ram_stylesheet_change_ready__signal(self.ram.setStyleSheet)
        self.jetson_emmc_update_ready__signal.connect(self.emmc.setText)
        self.jetson_emmc_stylesheet_change_ready__signal.connect(self.emmc.setStyleSheet)
        self.jetson_gpu_temp_update_ready__signal.connect(self.gpu_temp.setText)
        self.jetson_gpu_temp_stylesheet_change_ready__signal(self.gpu_temp.setStyleSheet)
        self.bogie_connection_1_stylesheet_change_ready__signal.connect(self.bogie_right.setStyleSheet)
        self.bogie_connection_2_stylesheet_change_ready__signal.connect(self.bogie_left.setStyleSheet)
        self.bogie_connection_3_stylesheet_change_ready__signal.connect(self.bogie_rear.setStyleSheet)
        self.camera_zed_stylesheet_change_ready__signal.connect(self.zed.setStyleSheet)
        self.camera_under_stylesheet_change_ready__signal.connect(self.under_cam.setStyleSheet)
        self.camera_chassis_stylesheet_change_ready__signal.connect(self.chassis_cam.setStyleSheet)
        self.camera_main_stylesheet_change_ready__signal.connect(self.main_cam.setStyleSheet)
        self.gps_stylesheet_change_ready__signal.connect(self.gps.setStyleSheet)
        self.frsky_stylesheet_change_ready__signal.connect(self.frsky.setStyleSheet)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False


if __name__ == '__main__':
    rover_statuses = SensorCore()
    rover_statuses.run()
