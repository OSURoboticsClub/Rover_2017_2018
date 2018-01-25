import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy
import time
from cv_bridge import CvBridge, CvBridgeError
import cv2
import qimage2ndarray
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
#from sensor_msgs.msg import Image, CompressedImage


class DriveTest(QtCore.QThread):
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

    def setup_start_and_kill_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.not_abort = False
