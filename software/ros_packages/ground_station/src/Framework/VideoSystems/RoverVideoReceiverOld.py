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


class VideoTest(QtCore.QThread):
    publish_message_signal = QtCore.pyqtSignal()
    image_ready_signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects, screen_label, video_size=None, sub_path=None):
        super(VideoTest, self).__init__()

        self.not_abort = True

        self.shared_objects = shared_objects

        self.right_screen_label = screen_label  # type: QtGui.QPixmap
        self.video_size = video_size

        self.message = None

        self.publisher = rospy.Subscriber(sub_path, CompressedImage, self.__receive_message)

        self.raw_image = None
        self.cv_image = None
        self.pixmap = None
        self.bridge = CvBridge()
        # self.bridge.com

        self.new_frame = False
        self.frame_count = 0
        self.last_frame_time = time.time()
        self.fps = 0

        self.name = sub_path.split("/")[2].replace("_", " ").title()

        self.font = cv2.FONT_HERSHEY_TRIPLEX

        thickness = 1
        baseline = 0

        text_size = cv2.getTextSize(self.name, self.font, thickness, baseline)
        print text_size

        text_width, text_height = text_size[0]

        width = text_width + 10
        height = text_height + 20

        self.blank_image = np.zeros((height, width, 3), np.uint8)
        cv2.putText(self.blank_image, self.name, ((width - text_width) / 2, int((height * 2) / 3)), self.font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        self.blank_image = cv2.resize(self.blank_image, (width / 2, height / 2))

    def run(self):
        # TODO: Thread starting message here
        y_offset = 0
        x_offset = 0

        while self.not_abort:
            if self.raw_image and self.new_frame:
                self.cv_image = self.bridge.compressed_imgmsg_to_cv2(self.raw_image, "rgb8")

                self.cv_image = self.__show_fps(self.cv_image)

                self.cv_image[y_offset:y_offset + self.blank_image.shape[0], x_offset:x_offset + self.blank_image.shape[1]] = self.blank_image

                if self.video_size:
                    self.cv_image = cv2.resize(self.cv_image, self.video_size)
                self.pixmap = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(self.cv_image))
                self.image_ready_signal.emit()
                self.new_frame = False

            if (time.time() - self.last_frame_time) >= 0.5:
                self.fps = int(self.frame_count / (time.time() - self. last_frame_time))

                self.last_frame_time = time.time()
                self.frame_count = 0

            self.msleep(18)
        # TODO: Thread ending message here

    def __show_fps(self, image):
        thickness = 1
        baseline = 0

        fps_string = str(self.fps)

        text_size = cv2.getTextSize(fps_string, self.font, thickness, baseline)

        text_width, text_height = text_size[0]

        width = text_width + 10
        height = text_height + 20

        fps_image = np.zeros((height, width, 3), np.uint8)

        cv2.putText(fps_image, fps_string, ((width - text_width) / 2, int((height * 2) / 3)), self.font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        fps_image = cv2.resize(fps_image, (width / 2, height / 2))

        y_offset = 0
        x_offset = (image.shape[1] - fps_image.shape[1]) / 2

        image[y_offset:y_offset + fps_image.shape[0], x_offset:x_offset + fps_image.shape[1]] = fps_image

        return image

    def __on_image_update_ready(self):
        self.right_screen_label.setPixmap(self.pixmap)

    def __receive_message(self, message):
        self.raw_image = message
        self.new_frame = True
        self.frame_count += 1

    def connect_signals_and_slots(self):
        self.image_ready_signal.connect(self.__on_image_update_ready)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.not_abort = False