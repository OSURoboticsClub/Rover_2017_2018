# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from time import time


#####################################
# Global Variables
#####################################
THREAD_HERTZ = 5


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class BashConsole(QtCore.QThread):
    def __init__(self, shared_objects):
        super(BashConsole, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.ssh_widget = self.left_screen.ssh_console_widget  # type: QtWidgets.QSpinBox

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

    def run(self):
        while self.run_thread_flag:
            start_time = time()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
