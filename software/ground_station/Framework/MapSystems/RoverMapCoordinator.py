#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
from PIL.ImageQt import ImageQt

import logging

import rospy

# Custom Imports
import RoverMap

#####################################
# Global Variables
#####################################
# put some stuff here later so you can remember


class RoverMapCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal()

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
        self.overlay_image_object = None

        self.map_pixmap = None

    def run(self):
        self.logger.debug("Starting Map Coordinator Thread")

        while self.run_thread_flag:
            if self.setup_map_flag:
                self._map_setup()
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

    def _map_setup(self):
        self.google_maps_object = RoverMap.GMapsStitcher(1280,
                                                         720,
                                                         44.567161,
                                                         -123.278432,
                                                         18,
                                                         'terrain',
                                                         None, 20)
        self.overlay_image_object = (
            RoverMap.OverlayImage(44.567161, -123.278432,
                                  self.google_maps_object.northwest,
                                  self.google_maps_object.southeast,
                                  self.google_maps_object.big_image[0],
                                  self.google_maps_object.big_image[1],
                                  1280, 720))

    def _get_map_image(self):
        self.map_image = self.google_maps_object.display_image
        self.map_image.paste(self.overlay_image_object.display_image)
        # get overlay here
        qim = ImageQt(self.map_image)
        self.map_pixmap = QtGui.QPixmap.fromImage(qim)
        self.pixmap_ready_signal.emit()

    def connect_signals_and_slots(self):
        self.pixmap_ready_signal.connect(self.pixmap_ready__slot)

    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False

    def setup_signals(self, start_signal, signals_and_slots_signal,
                      kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested_slot)

    def pixmap_ready__slot(self):
        self.logger.info("Made it")
        self.mapping_label.setPixmap(self.map_pixmap)
