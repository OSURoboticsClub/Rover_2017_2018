#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtGui, QtWidgets
import logging
import cv2
import numpy as np
import qimage2ndarray
import pprint

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

# Custom Imports
import RoverVideoReceiver

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras"
EXCLUDED_CAMERAS = ["zed"]


#####################################
# RoverVideoCoordinator Class Definition
#####################################
class RoverVideoCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(RoverVideoCoordinator, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.primary_video_display_label = self.right_screen.primary_video_label  # type:QtWidgets.QLabel
        self.secondary_video_display_label = self.right_screen.secondary_video_label  # type:QtWidgets.QLabel
        self.tertiary_video_display_label = self.right_screen.tertiary_video_label  # type:QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.setup_cameras_flag = True

        # ########## Class Variables ##########
        # Camera variables
        self.camera_threads = {}
        self.valid_cameras = []

        # Setup cameras
        self.__get_cameras()
        self.__setup_video_threads()

        self.primary_label_current_setting = 0
        self.secondary_label_current_setting = 1
        self.tertiary_label_current_setting = 0

        self.camera_data = {x: {"new_opencv": False} for x in self.valid_cameras}
        print self.camera_data

    def run(self):
        self.logger.debug("Starting Video Coordinator Thread")

        while self.run_thread_flag:

            self.msleep(100)

        self.__wait_for_camera_threads()

        self.logger.debug("Stopping Video Coordinator Thread")

    def __get_cameras(self):
        topics = rospy.get_published_topics(CAMERA_TOPIC_PATH)

        names = []

        for topics_group in topics:
            main_topic = topics_group[0]
            camera_name = main_topic.split("/")[2]
            names.append(camera_name)

        names = set(names)

        for camera in EXCLUDED_CAMERAS:
            if camera in names:
                names.remove(camera)

        self.valid_cameras = list(names)

    def __setup_video_threads(self):
        for camera in self.valid_cameras:
            self.camera_threads[camera] = RoverVideoReceiver.RoverVideoReceiver(camera)

    def __wait_for_camera_threads(self):
        for camera in self.camera_threads:
            self.camera_threads[camera].wait()

    def connect_signals_and_slots(self):
        for thread in self.camera_threads:
            self.camera_threads[thread].image_ready_signal.connect(self.pixmap_ready__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

        for camera in self.camera_threads:
            self.camera_threads[camera].setup_signals(start_signal, signals_and_slots_signal, kill_signal)

    def pixmap_ready__slot(self, camera):
        if self.valid_cameras[self.primary_label_current_setting] == camera:
            self.primary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_1280x720_image)

        if self.valid_cameras[self.secondary_label_current_setting] == camera:
            self.secondary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)

        if self.valid_cameras[self.tertiary_label_current_setting] == camera:
            self.tertiary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False

