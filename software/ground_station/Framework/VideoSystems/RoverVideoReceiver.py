#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtGui, QtWidgets
import logging
import cv2
import numpy as np
import qimage2ndarray

import rospy
import dynamic_reconfigure.client
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

# Custom Imports

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras/"

QUALITY_MAX = 80
QUALITY_MIN = 15


#####################################
# RoverVideoReceiver Class Definition
#####################################
class RoverVideoReceiver(QtCore.QThread):
    image_ready_signal = QtCore.pyqtSignal(str)

    def __init__(self, camera_name):
        super(RoverVideoReceiver, self).__init__()

        # ########## Reference to class init variables ##########
        self.camera_name = camera_name

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.camera_title_name = self.camera_name.replace("_", " ").title()

        self.topic_base_path = CAMERA_TOPIC_PATH + self.camera_name
        self.camera_topics = {}

        self.current_max_resolution = None

        self.current_camera_settings = {
            "resolution": None,
            "quality_setting": QUALITY_MAX,

        }

        self.previous_camera_settings = self.current_camera_settings.copy()

        self.temp_topic_path = CAMERA_TOPIC_PATH + self.camera_name + "/image_640x360/compressed"

        # Subscription variables
        self.video_subscriber = None  # type: rospy.Subscriber

        # Image variables
        self.raw_image = None
        self.opencv_1280x720_image = None
        self.opencv_640x360_image = None

        self.pixmap_1280x720_image = None
        self.pixmap_640x360_image = None

        # Processing variables
        self.bridge = CvBridge()  # OpenCV ROS Video Data Processor
        self.video_enabled = True
        self.new_frame = False

        # Text Drawing Variables
        self.font = cv2.FONT_HERSHEY_TRIPLEX
        self.font_thickness = 1
        self.font_baseline = 0

        self.camera_name_opencv_1280x720_image = None
        self.camera_name_opencv_640x360_image = None

        # ROS Dynamic Reconfigure Client
        self.reconfigure_clients = {}

        # Initial class setup to make text images and get camera resolutions
        self.__create_camera_name_opencv_images()
        self.__get_camera_available_resolutions()
        self.__setup_reconfigure_clients()

    def run(self):
        self.logger.debug("Starting \"%s\" Camera Thread" % self.camera_title_name)

        while self.run_thread_flag:
            if self.video_enabled:
                self.__show_video_enabled()
            else:
                self.__show_video_disabled()

            self.msleep(10)

        self.logger.debug("Stopping \"%s\" Camera Thread" % self.camera_title_name)

    def __perform_quality_check_and_adjust(self):
        self.__set_jpeg_quality(self.current_camera_settings["quality_setting"])

    def __set_jpeg_quality(self, quality_setting):
        self.reconfigure_clients[self.current_camera_settings["resolution"]].update_configuration({"jpeg_quality": quality_setting})

    def __setup_reconfigure_clients(self):
        for resolution_group in self.camera_topics:
            image_topic_string = "image_%sx%s" % resolution_group
            full_topic = self.topic_base_path + "/" + image_topic_string + "/compressed"
            self.reconfigure_clients[resolution_group] = dynamic_reconfigure.client.Client(full_topic)

    def __get_camera_available_resolutions(self):
        topics = rospy.get_published_topics(self.topic_base_path)

        resolution_options = []

        for topics_group in topics:
            main_topic = topics_group[0]
            camera_name = main_topic.split("/")[3]
            resolution_options.append(camera_name)

        resolution_options = list(set(resolution_options))

        for resolution in resolution_options:
            # Creates a tuple in (width, height) format that we can use as the key
            group = int(resolution.split("image_")[1].split("x")[0]), int(resolution.split("image_")[1].split("x")[1])
            self.camera_topics[group] = self.topic_base_path + "/" + resolution + "/compressed"

    def __update_camera_subscription_and_settings(self):
        if self.current_camera_settings["resolution"] != self.previous_camera_settings["resolution"]:

            if self.video_subscriber:
                self.video_subscriber.unregister()
            new_topic = self.camera_topics[self.current_camera_settings["resolution"]]
            self.video_subscriber = rospy.Subscriber(new_topic, CompressedImage, self.__image_data_received_callback)

            self.new_frame = False
            while not self.new_frame:
                self.msleep(10)

            self.previous_camera_settings["resolution"] = self.current_camera_settings["resolution"]

    def __show_video_enabled(self):
        self.__update_camera_subscription_and_settings()

        if self.new_frame and self.current_camera_settings["resolution"]:
            self.__perform_quality_check_and_adjust()

            opencv_image = self.bridge.compressed_imgmsg_to_cv2(self.raw_image, "rgb8")

            self.__create_final_pixmaps(opencv_image)

            self.image_ready_signal.emit(self.camera_name)
            self.new_frame = False

    def __show_video_disabled(self):
        blank_frame = np.zeros((720, 1280, 3), np.uint8)

        self.__create_final_pixmaps(blank_frame)

        self.image_ready_signal.emit(self.camera_name)

    def __create_final_pixmaps(self, opencv_image):
        height, width, _ = opencv_image.shape

        if width != 1280 and height != 720:
            self.opencv_1280x720_image = cv2.resize(opencv_image, (1280, 720))
        else:
            self.opencv_1280x720_image = opencv_image

        if width != 640 and height != 360:
            self.opencv_640x360_image = cv2.resize(opencv_image, (640, 360))
        else:
            self.opencv_640x360_image = opencv_image

        self.__apply_camera_name(self.opencv_1280x720_image, self.camera_name_opencv_1280x720_image)
        self.__apply_camera_name(self.opencv_640x360_image, self.camera_name_opencv_640x360_image)

        self.pixmap_1280x720_image = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(
            self.opencv_1280x720_image))
        self.pixmap_640x360_image = QtGui.QPixmap.fromImage(qimage2ndarray.array2qimage(
            self.opencv_640x360_image))

    def __image_data_received_callback(self, raw_image):
        self.raw_image = raw_image
        self.new_frame = True

    def __create_camera_name_opencv_images(self):
        camera_name_text_width, camera_name_text_height = \
            cv2.getTextSize(self.camera_title_name, self.font, self.font_thickness, self.font_baseline)[0]

        camera_name_width_buffered = camera_name_text_width + 10
        camera_name_height_buffered = camera_name_text_height + 20

        camera_name_opencv_image = np.zeros(
            (camera_name_height_buffered, camera_name_width_buffered, 3), np.uint8)

        cv2.putText(
            camera_name_opencv_image,
            self.camera_title_name,
            ((camera_name_width_buffered - camera_name_text_width) / 2, int((camera_name_height_buffered * 2) / 3)),
            self.font,
            1,
            (255, 255, 255),
            1,
            cv2.LINE_AA)

        self.camera_name_opencv_1280x720_image = \
            cv2.resize(camera_name_opencv_image, (camera_name_width_buffered, camera_name_height_buffered))

        self.camera_name_opencv_640x360_image = \
            cv2.resize(camera_name_opencv_image, (camera_name_width_buffered / 2, camera_name_height_buffered / 2))

    def change_max_resolution_setting(self, resolution_max):
        self.current_max_resolution = resolution_max
        self.current_camera_settings["resolution"] = resolution_max

    def toggle_video_display(self):
        if self.video_enabled:
            if self.video_subscriber:
                self.video_subscriber.unregister()
            self.new_frame = True
            self.video_enabled = False
        else:
            new_topic = self.camera_topics[self.current_camera_settings["resolution"]]
            self.video_subscriber = rospy.Subscriber(new_topic, CompressedImage, self.__image_data_received_callback)
            self.video_enabled = True

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False

    @staticmethod
    def __apply_camera_name(opencv_image, font_opencv_image):
        opencv_image[0:0 + font_opencv_image.shape[0], 0:0 + font_opencv_image.shape[1]] = \
            font_opencv_image
