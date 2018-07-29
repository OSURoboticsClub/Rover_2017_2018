# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from time import time
import paramiko

#####################################
# Global Variables
#####################################
ACCESS_POINT_IP = "192.168.1.20"  # The channel only has to be set on the access point. The staion will adjust.
ACCESS_POINT_USER = "ubnt"
ACCESS_POINT_PASSWORD = "rover4lyfe^"  # We don't care about this password, don't freak out. Wifi is open anyways...


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class SSHConsole(QtCore.QThread):
    def __init__(self, shared_objects):
        super(SSHConsole, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.ubiquiti_channel_spin_box = self.left_screen.ssh_console_widget  # type: QtWidgets.QSpinBox
        self.ubiquiti_channel_apply_button = self.left_screen.ubiquiti_channel_apply_button  # type: QtWidgets.QPushButton

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########

        self.connect_signals_and_slots()

    def connect_signals_and_slots(self):
        pass