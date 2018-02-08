#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging

import rospy

# Custom Imports
import RoverMap

#####################################
# Global Variables
#####################################
# put some stuff here later so you can remember


class RoverMapCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(RoverMapCoordinator, self).init()

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]
        self.mapping_label = self.left_screen.mapping_label

        self.setings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

        self.run_thread_flag = True
        self.setup_map_flag = True

        # setup map
        self._setup_map_threads()

    def run(self):
        self.logger.debug("Starting Map Coordinator Thread")
        
        while self.run_thread_flag:
            self.msleep(10)

        self.__wait_for_map_thread()
        self.logger.debug("Stopping Map Coordinator Thread")

    def __wait_for_map_thread(self):
        self.map_thread.wait()

    def _setup_map_threads(self):
        self.map_thread = RoverMap.GMapsStitcher(1280, 
                                                 720, 44.567161, -123.278432,
                                                 18, 'terrain', None, 20)
    
    def pixmap_ready_slot(self):
        self.mapping_label.setPixmap(self.map_thread.display_image)

    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False