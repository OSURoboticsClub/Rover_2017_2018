#!/usr/bin/env python

"""
    Main file used to launch the Rover Base Station
    No other files should be used for launching this application.
"""

#####################################
# Imports
#####################################
# Python native imports
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy
import roslaunch
import os
import psutil
import subprocess
from cv_bridge import CvBridge, CvBridgeError
import cv2
import qimage2ndarray

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
#from sensor_msgs.msg import Image, CompressedImage
# Custom Imports

#####################################
# Global Variables
#####################################
UI_FILE_LEFT = "resources/ui/left_screen.ui"
UI_FILE_RIGHT = "resources/ui/right_screen.ui"

#####################################
# Class Organization
#####################################
# Class Name:
#   "init"
#   "private methods"
#   "public methods, minus slots"
#   "slot methods"
#   "static methods"


class VideoTest(QtCore.QThread):
    ROS_CORE_COMMAND = ["roscore"]

    publish_message_signal = QtCore.pyqtSignal()
    image_ready_signal = QtCore.pyqtSignal()

    def __init__(self, screen_label, video_size=None, sub_path=None):
        super(VideoTest, self).__init__()

        self.not_abort = True

        self.right_screen_label = screen_label  # type: QtGui.QPixmap
        self.video_size = video_size

        #rospy.init_node("video_test")

        self.message = None

        self.publisher = rospy.Subscriber(sub_path, CompressedImage, self.__receive_message)

        self.raw_image = None
        self.cv_image = None
        self.pixmap = None
        self.bridge = CvBridge()
        # self.bridge.com

        self.image_ready_signal.connect(self.__on_image_update_ready)

    def run(self):
        # TODO: Thread starting message here
        while self.not_abort:
            if self.raw_image:
                self.cv_image = self.bridge.compressed_imgmsg_to_cv2(self.raw_image, "rgb8")

                if self.video_size:
                    self.cv_image = cv2.resize(self.cv_image, self.video_size)
                self.pixmap = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(self.cv_image))
                self.image_ready_signal.emit()
                #print "doin stuff"
            self.msleep(10)
        # TODO: Thread ending message here

    def __on_image_update_ready(self):
        self.right_screen_label.setPixmap(self.pixmap)

    def __receive_message(self, message):
        #print "messgae"
        self.raw_image = message

    def setup_start_and_kill_signals(self, start_signal, kill_signal):
        start_signal.connect(self.start)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.not_abort = False


class DriveTest(QtCore.QThread):
    ROS_CORE_COMMAND = ["roscore"]

    publish_message_signal = QtCore.pyqtSignal()

    def __init__(self):
        super(DriveTest, self).__init__()

        self.not_abort = True

        self.message = None
        self.publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.init_node("test")

    def run(self):
        # TODO: Thread starting message here
        while self.not_abort:
            self.message = Twist()

            self.message.linear.x = 1.0
            self.message.angular.z = 1.0

            self.publisher.publish(self.message)

            self.msleep(100)
        # TODO: Thread ending message here

    def __publish_message(self):
        pass

    def setup_start_and_kill_signals(self, start_signal, kill_signal):
        start_signal.connect(self.start)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.not_abort = False


#####################################
# ApplicationWindow Class Definition
#####################################
class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None, ui_file_path=None):
        super(ApplicationWindow, self).__init__(parent)

        uic.loadUi(ui_file_path, self)

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


#####################################
# GroundStation Class Definition
#####################################
class GroundStation(QtCore.QObject):
    LEFT_SCREEN_ID = 0
    RIGHT_SCREEN_ID = 1

    start_threads_signal = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        self.left_screen = self.create_application_window(UI_FILE_LEFT, "Rover Ground Station Left Screen",
                                                          self.LEFT_SCREEN_ID)  # type: ApplicationWindow
        self.right_screen = self.create_application_window(UI_FILE_RIGHT, "Rover Ground Station Right Screen",
                                                           self.RIGHT_SCREEN_ID)  # type: ApplicationWindow

        # Start ROSCORE
        self.video_test = VideoTest(self.right_screen.primary_video_label, sub_path="/cam1/usb_cam1/image_raw/compressed")
        self.video_test_1 = VideoTest(self.right_screen.secondary_video_label, (640, 360), sub_path="/cam2/usb_cam2/image_raw//compressed")
        self.video_test_2 = VideoTest(self.right_screen.tertiary_video_label, (640, 360), sub_path="/zed/right/image_raw_color/compressed")
        self.drive_test = DriveTest()

        # Keep track of all threads
        self.threads = []
        self.threads.append(self.drive_test)
        self.threads.append(self.video_test)
        self.threads.append(self.video_test_1)
        self.threads.append(self.video_test_2)

        # Connect signals
        for thread in self.threads:
            thread.setup_start_and_kill_signals(self.start_threads_signal, self.kill_threads_signal)

        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def __connect_signals_to_slots(self):
        self.left_screen.exit_requested_signal.connect(self.on_exit_requested__slot)
        self.right_screen.exit_requested_signal.connect(self.on_exit_requested__slot)

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        # Wait for Threads
        for thread in self.threads:
            thread.wait()

        QtGui.QGuiApplication.exit()

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        app_window = ApplicationWindow(parent=None, ui_file_path=ui_file_path)  # Make a window in this application
        app_window.setWindowTitle(title)  # Sets the window title

        app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
                                  QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
                                  QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
                                  QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        app_window.showFullScreen()  # Shows the window in full screen mode

        return app_window


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work properly

    application = QtWidgets.QApplication(sys.argv)  # Create the ase qt gui application

    ground_station = GroundStation()

    application.exec_()  # Execute launching of the application
