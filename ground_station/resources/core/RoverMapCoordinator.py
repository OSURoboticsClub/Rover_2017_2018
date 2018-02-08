#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging

import rospy

# Custom Imports
import MapHelper

#####################################
# Global Variables
#####################################
# put some stuff here later so you can remember


class RoverMapCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(RoverMapCoordinator, self).init()

        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["left_screen"]