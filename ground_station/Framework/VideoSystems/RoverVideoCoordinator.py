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

#####################################
# Global Variables
#####################################
FONT = cv2.FONT_HERSHEY_TRIPLEX


#####################################
# RoverVideoReceiver Class Definition
#####################################
class RoverVideoReceiver(QtCore.QThread):
    def __init__(self, shared_objects):
        super(RoverVideoReceiver, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.primary_video_display_label = self.right_screen.primary_video_label  # type:QtWidgets.QLabel
        self.primary_video_display_label = self.right_screen.secondary_video_label  # type:QtWidgets.QLabel
        self.primary_video_display_label = self.right_screen.tertiary_video_label  # type:QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

    def run(self):
        self.logger.debug("Starting Video Coordinator Thread")

        while self.run_thread_flag:

            self.msleep(100)

        self.logger.debug("Stopping Video Coordinator Thread")

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False