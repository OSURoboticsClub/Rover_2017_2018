#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
from PIL.ImageQt import ImageQt
from PIL import Image

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
    change_waypoint_signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(RoverMapCoordinator, self).__init__()

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]
        self.mapping_label = self.left_screen.mapping_label
        self.navigation_label = self.left_screen.tableWidget
        self.landmark_label = self.left_screen.tableWidget_2

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
            self.msleep(30)

        self.logger.debug("Stopping Map Coordinator Thread")

    # def _setup_map_threads(self):
    #     self.google_maps_object = RoverMap.GMapsStitcher(1280,
    #                                                      720,
    #                                                      44.567161,
    #                                                      -123.278432,
    #                                                      18,
    #                                                      'satellite',
    #                                                      None, 20)

    def _map_setup(self):
        self.google_maps_object = RoverMap.GMapsStitcher(1280,
                                                         720,
                                                         44.567161,
                                                         -123.278432,
                                                         18,
                                                         'satellite',
                                                         None, 20)
        self.overlay_image_object = (
            RoverMap.OverlayImage(44.567161, -123.278432,
                                  self.google_maps_object.northwest,
                                  self.google_maps_object.southeast,
                                  self.google_maps_object.big_image.size[0],
                                  self.google_maps_object.big_image.size[1],
                                  1280, 720))

    def _get_map_image(self):
        while self.map_image is None:
            self.map_image = self.google_maps_object.display_image
        # self.overlay_image_object.update_new_location(44.567161,
        #                                               -123.278432,
        #                                               .7,
        #                                               [],
        #                                               [])
        self.update_overlay()
        self.map_image.paste(self.overlay_image_object.display_image,
                             (0, 0),
                             self.overlay_image_object.display_image)
        # self.map_image = Image.alpha_composite(
        #                   self.google_maps_object.display_image,
        #                   self.overlay_image_object.display_image)
        # get overlay here
        qim = ImageQt(self.map_image)
        self.map_pixmap = QtGui.QPixmap.fromImage(qim)
        self.pixmap_ready_signal.emit()

    def connect_signals_and_slots(self):
        self.pixmap_ready_signal.connect(self.pixmap_ready__slot)
        self.change_waypoint_signal.connect(self.update_overlay)

    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False

    def setup_signals(self, start_signal, signals_and_slots_signal,
                      kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested_slot)

    def pixmap_ready__slot(self):
        self.mapping_label.setPixmap(self.map_pixmap)

    def _get_table_elements(self, UI_element):
        temp_list = []
        count = UI_element.rowCount()
        for row in range(0, count):
            num = UI_element.item(row, 0).text()
            lat = float(UI_element.item(row, 1).text())
            lng = float(UI_element.item(row, 2).text())
            temp_tuple = (num, lat, lng)
            temp_list.append(temp_tuple)
        return temp_list

    def update_overlay(self):
        longitude = 44.567161
        latitude = -123.278432
        navigation_list = self._get_table_elements(self.navigation_label)
        # landmark_list = self._get_table_elements(self.landmark_label)
        landmark_list = []
        self.overlay_image = self.overlay_image_object.update_new_location(latitude,
                                                      longitude,
                                                      70,
                                                      navigation_list,
                                                      landmark_list)
        # self.overlay_image.save("something.png")
