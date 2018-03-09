#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging

import rospy

# Custom Imports
import RoverVideoReceiver
from rover_camera.msg import CameraControlMessage

#####################################
# Global Variables
#####################################
CAMERA_TOPIC_PATH = "/cameras/"
EXCLUDED_CAMERAS = ["zed"]

PRIMARY_LABEL_MAX = (640, 360)
SECONDARY_LABEL_MAX = (640, 360)
TERTIARY_LABEL_MAX = (640, 360)


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

        self.index_to_label_element = {
            0: self.primary_video_display_label,
            1: self.secondary_video_display_label,
            2: self.tertiary_video_display_label
        }

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

        reset_camera_message = CameraControlMessage()
        reset_camera_message.enable_small_broadcast = True

        # Reset default cameras
        rospy.Publisher("/cameras/chassis/camera_control", CameraControlMessage, queue_size=1).publish(reset_camera_message)
        rospy.Publisher("/cameras/undercarriage/camera_control", CameraControlMessage, queue_size=1).publish(reset_camera_message)
        rospy.Publisher("/cameras/main_navigation/camera_control", CameraControlMessage, queue_size=1).publish(reset_camera_message)
        rospy.Publisher("/cameras/end_effector/camera_control", CameraControlMessage, queue_size=1).publish(reset_camera_message)

        # self.msleep(3000)

        # Setup cameras
        self.__get_cameras()
        self.__setup_video_threads()

        self.primary_label_current_setting = 0
        self.secondary_label_current_setting = min(self.primary_label_current_setting + 1, len(self.valid_cameras))
        self.tertiary_label_current_setting = min(self.secondary_label_current_setting + 1, len(self.valid_cameras))

        self.index_to_label_current_setting = {
            0, self.primary_label_current_setting,
            1, self.secondary_label_current_setting,
            2, self.tertiary_label_current_setting
        }

        self.current_label_for_joystick_adjust = 0
        self.gui_selection_update_needed = True

        self.set_max_resolutions_flag = True

        self.first_image_received = False

    def run(self):
        self.logger.debug("Starting Video Coordinator Thread")

        while self.run_thread_flag:
            self.__set_max_resolutions()
            self.__toggle_background_cameras_if_needed()
            self.__update_gui_element_selection()
            self.msleep(10)

        self.__wait_for_camera_threads()

        self.logger.debug("Stopping Video Coordinator Thread")

    def __set_max_resolutions(self):
        if self.set_max_resolutions_flag:
            self.camera_threads[self.valid_cameras[self.primary_label_current_setting]].set_hard_max_resolution(PRIMARY_LABEL_MAX)

            if self.secondary_label_current_setting != self.primary_label_current_setting:
                self.camera_threads[self.valid_cameras[self.secondary_label_current_setting]].set_hard_max_resolution(SECONDARY_LABEL_MAX)

            if self.tertiary_label_current_setting != self.primary_label_current_setting and self.tertiary_label_current_setting != self.secondary_label_current_setting:
                self.camera_threads[self.valid_cameras[self.tertiary_label_current_setting]].set_hard_max_resolution(SECONDARY_LABEL_MAX)

            self.set_max_resolutions_flag = False

    def __toggle_background_cameras_if_needed(self):
        enabled = list({self.primary_label_current_setting, self.secondary_label_current_setting,
                        self.tertiary_label_current_setting})

        for camera_index, camera_name in enumerate(self.valid_cameras):
            if camera_index not in enabled and camera_index not in self.disabled_cameras and self.camera_threads[camera_name].video_enabled:
                self.camera_threads[camera_name].toggle_video_display()
            elif camera_index in enabled and camera_index not in self.disabled_cameras and not self.camera_threads[camera_name].video_enabled:
                self.camera_threads[camera_name].toggle_video_display()

    def __update_gui_element_selection(self):
        if self.gui_selection_update_needed:
            elements_to_reset = range(len(self.valid_cameras))
            elements_to_reset.remove(self.current_label_for_joystick_adjust)

            for index in elements_to_reset:
                self.index_to_label_element[index].setStyleSheet("background-color: transparent;")

            self.index_to_label_element[self.current_label_for_joystick_adjust].setStyleSheet("border: 2px solid orange")

            self.gui_selection_update_needed = False

    def __get_cameras(self):
        topics = rospy.get_published_topics(CAMERA_TOPIC_PATH)

        names = []

        for topics_group in topics:
            main_topic = topics_group[0]
            if "heartbeat" in main_topic:
                continue
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

        self.shared_objects["threaded_classes"]["Joystick Sender"].change_gui_element_selection__signal.connect(
            self.on_camera_gui_element_selection_changed)
        self.shared_objects["threaded_classes"]["Joystick Sender"].change_camera_selection__signal.connect(
            self.on_camera_selection_for_current_gui_element_changed)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

        for camera in self.camera_threads:
            self.camera_threads[camera].setup_signals(start_signal, signals_and_slots_signal, kill_signal)

    def __change_display_source_primary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.primary_label_current_setting = (self.primary_label_current_setting + 1) % len(self.valid_cameras)
            self.set_max_resolutions_flag = True
        elif event.button() == QtCore.Qt.RightButton:
            if self.primary_label_current_setting in self.disabled_cameras:
                self.disabled_cameras.remove(self.primary_label_current_setting)
            else:
                self.disabled_cameras.append(self.primary_label_current_setting)
            self.camera_threads[self.valid_cameras[self.primary_label_current_setting]].toggle_video_display()

    def __change_display_source_secondary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.secondary_label_current_setting = (self.secondary_label_current_setting + 1) % len(self.valid_cameras)
            self.set_max_resolutions_flag = True
        elif event.button() == QtCore.Qt.RightButton:
            if self.secondary_label_current_setting in self.disabled_cameras:
                self.disabled_cameras.remove(self.secondary_label_current_setting)
            else:
                self.disabled_cameras.append(self.secondary_label_current_setting)
            self.camera_threads[self.valid_cameras[self.secondary_label_current_setting]].toggle_video_display()

    def __change_display_source_tertiary_mouse_press_event(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.tertiary_label_current_setting = (self.tertiary_label_current_setting + 1) % len(self.valid_cameras)
            self.set_max_resolutions_flag = True
        elif event.button() == QtCore.Qt.RightButton:
            if self.tertiary_label_current_setting in self.disabled_cameras:
                self.disabled_cameras.remove(self.tertiary_label_current_setting)
            else:
                self.disabled_cameras.append(self.tertiary_label_current_setting)
            self.camera_threads[self.valid_cameras[self.tertiary_label_current_setting]].toggle_video_display()

    def pixmap_ready__slot(self, camera):
        if self.valid_cameras[self.primary_label_current_setting] == camera:
            try:
                self.primary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_1280x720_image)
            except:
                pass

        if self.valid_cameras[self.secondary_label_current_setting] == camera:
            try:
                self.secondary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)
            except:
                pass

        if self.valid_cameras[self.tertiary_label_current_setting] == camera:
            try:
                self.tertiary_video_display_label.setPixmap(self.camera_threads[camera].pixmap_640x360_image)
            except:
                pass

    def on_camera_gui_element_selection_changed(self, direction):
        new_selection = self.current_label_for_joystick_adjust + direction

        if new_selection < 0:
            self.current_label_for_joystick_adjust = len(self.valid_cameras)
        elif new_selection > len(self.valid_cameras):
            self.current_label_for_joystick_adjust = 0
        else:
            self.current_label_for_joystick_adjust = new_selection

        self.gui_selection_update_needed = True

    def on_camera_selection_for_current_gui_element_changed(self, direction):
        new_label_setting = self.index_to_label_current_setting[self.current_label_for_joystick_adjust] + direction

        if new_label_setting < 0:
            self.index_to_label_current_setting[self.current_label_for_joystick_adjust] = len(self.valid_cameras)
        elif new_label_setting > len(self.valid_cameras):
            self.index_to_label_current_setting[self.current_label_for_joystick_adjust] = 0
        else:
            self.index_to_label_current_setting[self.current_label_for_joystick_adjust] = new_label_setting

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
