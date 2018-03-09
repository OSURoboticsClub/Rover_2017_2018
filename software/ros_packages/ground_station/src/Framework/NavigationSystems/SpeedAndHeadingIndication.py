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
THREAD_HERTZ = 10


#####################################
# Controller Class Definition
#####################################
class SpeedAndHeadingIndication(QtCore.QThread):
    show_compass_image__signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(SpeedAndHeadingIndication, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.heading_compass_label = self.right_screen.heading_compass_label  # type: QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.main_compass_image = PIL.Image.open("Resources/Images/compass.png").resize((300, 300))
        self.compass_pixmap = None

        self.current_rotation = 0

        # compass_image = PIL.Image.open("Resources/Images/compass.png").resize((300, 300)).rotate(45)  # PIL.Image
        # self.shared_objects["screens"]["right_screen"].heading_compass_label.setPixmap(QtGui.QPixmap.fromImage(ImageQt(compass_image)))

    def run(self):
        while self.run_thread_flag:

            start_time = time()
            self.current_rotation += 1

            new_compass_image = self.main_compass_image.rotate(self.current_rotation)

            self.compass_pixmap = QtGui.QPixmap.fromImage(ImageQt(new_compass_image))
            self.show_compass_image__signal.emit()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def on_new_compass_image_ready__slot(self):
        self.heading_compass_label.setPixmap(self.compass_pixmap)

    def connect_signals_and_slots(self):
        self.show_compass_image__signal.connect(self.on_new_compass_image_ready__slot)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
