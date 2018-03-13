# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from time import time
import PIL.Image
from PIL.ImageQt import ImageQt

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 20

ROTATION_SPEED_MODIFIER = 1


#####################################
# Controller Class Definition
#####################################
class SpeedAndHeadingIndication(QtCore.QThread):
    show_compass_image__signal = QtCore.pyqtSignal()
    heading_text_update_ready__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(SpeedAndHeadingIndication, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]

        self.heading_compass_label = self.right_screen.heading_compass_label  # type: QtWidgets.QLabel
        self.heading_text_label = self.right_screen.current_heading_label  # type: QtWidgets.QLabel
        self.next_goal_label = self.right_screen.next_goal_label  # type: QtWidgets.QLabel
        self.current_speed_label = self.right_screen.current_speed_label  # type: QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.main_compass_image = PIL.Image.open("Resources/Images/compass.png").resize((300, 300), PIL.Image.ANTIALIAS)
        self.compass_pixmap = None

        self.current_heading = 0
        self.current_heading_changed = True

        self.shown_heading = (self.current_heading + 1) % 360
        self.current_heading_shown_rotation_angle = 0
        self.rotation_direction = 1

        # compass_image = PIL.Image.open("Resources/Images/compass.png").resize((300, 300)).rotate(45)  # PIL.Image
        # self.shared_objects["screens"]["right_screen"].heading_compass_label.setPixmap(QtGui.QPixmap.fromImage(ImageQt(compass_image)))

    def run(self):
        self.on_heading_changed__slot(self.current_heading)

        while self.run_thread_flag:
            start_time = time()

            if self.current_heading_changed:
                self.update_heading_movement()
                self.current_heading_changed = False

            self.rotate_compass_if_needed()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def rotate_compass_if_needed(self):
        if int(self.shown_heading) != self.current_heading:
            self.shown_heading += self.rotation_direction * ROTATION_SPEED_MODIFIER
            self.shown_heading %= 360

            self.current_heading_shown_rotation_angle = int(self.shown_heading)

            new_compass_image = self.main_compass_image.rotate(self.current_heading_shown_rotation_angle, resample=PIL.Image.BICUBIC)

            self.compass_pixmap = QtGui.QPixmap.fromImage(ImageQt(new_compass_image))
            self.show_compass_image__signal.emit()

    def update_heading_movement(self):
        current_minus_shown = (self.current_heading - self.shown_heading) % 360

        if current_minus_shown >= 180:
            self.rotation_direction = -1
        else:
            self.rotation_direction = 1

    def on_heading_changed__slot(self, new_heading):
        self.current_heading = new_heading
        self.heading_text_update_ready__signal.emit(str(new_heading) + "°")
        self.current_heading_changed = True

    def __on_heading_clicked__slot(self, event):
        new_heading = self.current_heading
        if event.button() == QtCore.Qt.LeftButton:
            new_heading = (self.current_heading + 10) % 360
        elif event.button() == QtCore.Qt.RightButton:
            new_heading = (self.current_heading - 10) % 360

        self.on_heading_changed__slot(new_heading)
        self.heading_text_update_ready__signal.emit(str(new_heading) + "°")

    def on_new_compass_image_ready__slot(self):
        self.heading_compass_label.setPixmap(self.compass_pixmap)

    def connect_signals_and_slots(self):
        self.show_compass_image__signal.connect(self.on_new_compass_image_ready__slot)
        self.heading_text_update_ready__signal.connect(self.heading_text_label.setText)

        self.heading_compass_label.mousePressEvent = self.__on_heading_clicked__slot

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
