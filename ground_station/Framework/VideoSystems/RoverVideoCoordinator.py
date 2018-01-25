#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging

import rospy

# Custom Imports
import RoverVideoReceiver

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras/"
EXCLUDED_CAMERAS = ["zed"]

PRIMARY_LABEL_MAX = (640, 360)
SECONDARY_LABEL_MAX = (256, 144)
TERTIARY_LABEL_MAX = (256, 144)


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
        self.disabled_cameras = []

        # Setup cameras
        self.__get_cameras()
        self.__setup_video_threads()

        self.primary_label_current_setting = 0
        self.secondary_label_current_setting = 0
        self.tertiary_label_current_setting = 0

        self.primary_label_max_resolution = -1
        self.secondary_label_max_resolution = -1
        self.tertiary_label_max_resolution = -1

    def run(self):
        self.logger.debug("Starting Video Coordinator Thread")

        self.__set_max_resolutions()  # Do this initially so we don't try to disable cameras before they're set up
        self.msleep(500)

        while self.run_thread_flag:
            self.__set_max_resolutions()
            # self.__toggle_background_cameras_if_needed()
            self.msleep(10)

        self.__wait_for_camera_threads()

        self.logger.debug("Stopping Video Coordinator Thread")

    def __set_max_resolutions(self):
        self.primary_label_max_resolution = self.camera_threads[
            self.valid_cameras[self.primary_label_current_setting]].current_max_resolution
        self.secondary_label_max_resolution = self.camera_threads[
            self.valid_cameras[self.secondary_label_current_setting]].current_max_resolution
        self.tertiary_label_max_resolution = self.camera_threads[
            self.valid_cameras[self.tertiary_label_current_setting]].current_max_resolution

        if self.primary_label_max_resolution != PRIMARY_LABEL_MAX:
            self.camera_threads[self.valid_cameras[self.primary_label_current_setting]].change_max_resolution_setting(
                PRIMARY_LABEL_MAX)

        if self.secondary_label_max_resolution != SECONDARY_LABEL_MAX and self.secondary_label_current_setting != self.primary_label_current_setting:
            self.camera_threads[self.valid_cameras[self.secondary_label_current_setting]].change_max_resolution_setting(
                SECONDARY_LABEL_MAX)

        if self.tertiary_label_max_resolution != TERTIARY_LABEL_MAX and self.tertiary_label_current_setting != self.primary_label_current_setting:
            self.camera_threads[self.valid_cameras[self.tertiary_label_current_setting]].change_max_resolution_setting(
                TERTIARY_LABEL_MAX)

    def __toggle_background_cameras_if_needed(self):
        enabled = list({self.primary_label_current_setting, self.secondary_label_current_setting,
                        self.tertiary_label_current_setting})

        for camera_index, camera_name in enumerate(self.valid_cameras):
            if camera_index not in enabled:
                self.camera_threads[camera_name].toggle_video_display()
                self.disabled_cameras.append(camera_index)
            elif camera_index in self.disabled_cameras:
                self.camera_threads[camera_name].toggle_video_display()
                self.disabled_cameras.remove(camera_index)

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

        self.primary_video_display_label.mousePressEvent = self.__change_display_source_primary_mouse_press_event
        self.secondary_video_display_label.mousePressEvent = self.__change_display_source_secondary_mouse_press_event
        self.tertiary_video_display_label.mousePressEvent = self.__change_display_source_tertiary_mouse_press_event

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

        for camera in self.camera_threads:
            self.camera_threads[camera].setup_signals(start_signal, signals_and_slots_signal, kill_signal)

    def __change_display_source_primary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.primary_label_current_setting = (self.primary_label_current_setting + 1) % len(self.valid_cameras)
        elif event.button() == QtCore.Qt.RightButton:
            self.camera_threads[self.valid_cameras[self.primary_label_current_setting]].toggle_video_display()

    def __change_display_source_secondary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.secondary_label_current_setting = (self.secondary_label_current_setting + 1) % len(self.valid_cameras)
        elif event.button() == QtCore.Qt.RightButton:
            self.camera_threads[self.valid_cameras[self.secondary_label_current_setting]].toggle_video_display()

    def __change_display_source_tertiary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.tertiary_label_current_setting = (self.tertiary_label_current_setting + 1) % len(self.valid_cameras)
        elif event.button() == QtCore.Qt.RightButton:
            self.camera_threads[self.valid_cameras[self.tertiary_label_current_setting]].toggle_video_display()

    def pixmap_ready__slot(self, camera):
        if self.valid_cameras[self.primary_label_current_setting] == camera:
            self.primary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_1280x720_image)

        if self.valid_cameras[self.secondary_label_current_setting] == camera:
            self.secondary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)

        if self.valid_cameras[self.tertiary_label_current_setting] == camera:
            self.tertiary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
