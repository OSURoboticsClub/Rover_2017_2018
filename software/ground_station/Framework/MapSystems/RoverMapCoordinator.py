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
        super(RoverMapCoordinator, self).__init__()

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]
        self.mapping_label = self.left_screen.mapping_label

        self.setings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

        self.run_thread_flag = True
        self.setup_map_flag = True

        self.google_maps_object = None
        self.map_image = None
        self.overlay_image = None

        # setup map
        self._setup_map_threads()

    def run(self):
        self.logger.debug("Starting Map Coordinator Thread")
        
        while self.run_thread_flag:
            if self.setup_map_flag:
                self.__map_setup()
                self.setup_map_flag = False
            else:
                self._get_map_image()
            self.msleep(3)

        self.logger.debug("Stopping Map Coordinator Thread")

    def _setup_map_threads(self):
        self.google_maps_object = RoverMap.GMapsStitcher(1280, 
                                                         720,
                                                         44.567161,
                                                         -123.278432,
                                                         18,
                                                         'terrain',
                                                         None, 20)

    def _get_map_image(self):
        self.map_image = self.google_maps_object.display_image
        # get overlay here
        self.pixmap_ready_signal.emit()

    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False

    def setup_signals(self, start_signal, signals_and_slots_signal, 
                      kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested_slot)

    def connect_signals_and_slots(self):
        self.image_ready_signal.connect(self.pixmap_ready_slot)
