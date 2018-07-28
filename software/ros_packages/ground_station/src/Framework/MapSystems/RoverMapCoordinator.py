#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
from PIL.ImageQt import ImageQt
from PIL import Image
import PIL.ImageDraw
import PIL.Image
import PIL.ImageFont
import numpy

import logging

import rospy
from tf import transformations
from scipy.interpolate import interp1d
import math
from sensor_msgs.msg import Imu

# Custom Imports
import RoverMap
from Resources.Settings import Mapping as MappingSettings
from sensor_msgs.msg import NavSatFix

#####################################
# Global Variables
#####################################
# put some stuff here later so you can remember

GPS_POSITION_TOPIC = "/rover_odometry/fix"
IMU_DATA_TOPIC = "/rover_odometry/imu/data"


MAP_WIDGET_WIDTH = float(1280)
MAP_WIDGET_HEIGHT = float(720)
MAP_WIDGET_RATIO = MAP_WIDGET_WIDTH / MAP_WIDGET_HEIGHT


class RoverMapCoordinator(QtCore.QThread):
    pixmap_ready_signal = QtCore.pyqtSignal()
    change_waypoint_signal = QtCore.pyqtSignal()

    def __init__(self, shared_objects):
        super(RoverMapCoordinator, self).__init__()

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]
        self.mapping_label = self.left_screen.mapping_label  # type:QtWidgets.QLabel
        self.navigation_label = self.left_screen.navigation_waypoints_table_widget
        self.landmark_label = self.left_screen.landmark_waypoints_table_widget

        self.setings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

        self.gps_position_subscriber = rospy.Subscriber(GPS_POSITION_TOPIC, NavSatFix, self.gps_position_updated_callback)

        self.run_thread_flag = True
        self.setup_map_flag = True

        self.google_maps_object = None
        self.map_image = None
        self.map_image_copy = None
        self.overlay_image = None
        self.overlay_image_object = None

        self.map_pixmap = QtGui.QPixmap.fromImage(ImageQt(Image.open("Resources/Images/maps_loading.png").resize((1280, 720), Image.BICUBIC)))
        self.last_map_pixmap_cache_key = None

        self.longitude = None
        self.latitude = None
        self.last_heading = 0

        self.imu_data = None
        self.new_imu_data = False

        self.yaw = None
        self.pitch = None
        self.roll = None

        self.euler_interpolator = interp1d([math.pi, -math.pi], [-180, 180])

        self.imu_data_subscriber = rospy.Subscriber(IMU_DATA_TOPIC, Imu, self.on_imu_data_received)

        self.zoom_pan_x_location = 0
        self.zoom_pan_y_location = 0

        self.zoom_subtraction = 0

        self.x_drag = 0
        self.y_drag = 0

        self.x_drag_start = -1
        self.y_drag_start = -1

    def run(self):
        self.logger.debug("Starting Map Coordinator Thread")
        self.pixmap_ready_signal.emit()  # This gets us the loading map
        while self.run_thread_flag:
            if self.setup_map_flag:
                self._map_setup()
                self.setup_map_flag = False
            else:
                if self.new_imu_data:
                    self.calculate_euler_from_imu()
                    self.new_imu_data = False

                self._get_map_image()
            self.msleep(10)

        self.logger.debug("Stopping Map Coordinator Thread")

    def _map_setup(self):
        self.google_maps_object = RoverMap.GMapsStitcher(1280,
                                                         720,
                                                         44.5675721667,
                                                         -123.2750535,
                                                         18,  # FIXME: Used to be 18
                                                         'satellite',
                                                         None, 20)
        self.overlay_image_object = (
            RoverMap.OverlayImage(44.5675721667, -123.2750535,
                                  self.google_maps_object.northwest,
                                  self.google_maps_object.southeast,
                                  self.google_maps_object.big_image.size[0],
                                  self.google_maps_object.big_image.size[1],
                                  1280, 720))

    def _get_map_image(self):
        while self.map_image is None:
            self.map_image = self.google_maps_object.display_image

            if self.map_image:
                self.map_image_copy = self.map_image.copy()

        self.update_overlay()

        self.map_image = self.map_image_copy.copy()
        self.map_image.paste(self.overlay_image_object.display_image,
                             (0, 0),
                             self.overlay_image_object.display_image)

        # ##### Zoom testing! #####
        if self.zoom_subtraction:
            self.zoom_subtraction = self.constrain(self.zoom_subtraction, 0, 520)

            crop_x_start = ((self.zoom_subtraction * MAP_WIDGET_RATIO) / 2) - self.x_drag
            crop_y_start = (self.zoom_subtraction / 2) - self.y_drag
            crop_x_end = (MAP_WIDGET_WIDTH - ((self.zoom_subtraction * MAP_WIDGET_RATIO) / 2)) - self.x_drag
            crop_y_end = (MAP_WIDGET_HEIGHT - (self.zoom_subtraction / 2)) - self.y_drag
            crop_box = (int(crop_x_start), int(crop_y_start), int(crop_x_end), int(crop_y_end))

            self.map_image = self.map_image.crop(crop_box)
            self.map_image = self.map_image.resize((1280, 720), resample=PIL.Image.BICUBIC)

        # ##### Draw coordinates #####
        self._draw_coordinate_text(self.latitude, self.longitude)

        qim = ImageQt(self.map_image)
        self.map_pixmap = QtGui.QPixmap.fromImage(qim)

        # if self.map_pixmap.cacheKey() != self.last_map_pixmap_cache_key:
        #     self.last_map_pixmap_cache_key = self.map_pixmap.cacheKey()
        self.pixmap_ready_signal.emit()

    def _draw_coordinate_text(self, latitude, longitude):
        location_text = "LAT: %+014.9f\nLON: %+014.9f" % (latitude, longitude)

        font = PIL.ImageFont.truetype("UbuntuMono-R", size=20)

        new_image = PIL.Image.new('RGBA', (200, 45), "black")

        draw = PIL.ImageDraw.Draw(new_image)

        draw.multiline_text((5, 0), location_text, font=font)

        self.map_image.paste(new_image, (0, 0))

    def connect_signals_and_slots(self):
        self.pixmap_ready_signal.connect(self.pixmap_ready__slot)
        self.change_waypoint_signal.connect(self.update_overlay)

        self.mapping_label.mouseReleaseEvent = self.__mouse_released_event
        self.mapping_label.wheelEvent = self.__mouse_wheel_event
        self.mapping_label.mouseMoveEvent = self.__mouse_move_event

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
        if self.latitude and self.longitude:
            if not numpy.isnan(self.latitude) and not numpy.isnan(self.longitude):
                latitude = float(self.latitude)
                longitude = float(self.longitude)

                navigation_list = self._get_table_elements(self.navigation_label)
                landmark_list = self._get_table_elements(self.landmark_label)
                self.overlay_image = self.overlay_image_object.update_new_location(
                                                              latitude,
                                                              longitude,
                                                              self.last_heading,
                                                              navigation_list,
                                                              landmark_list)
                # self.last_heading = (self.last_heading + 5) % 360
                # self.overlay_image.save("something.png")

    def gps_position_updated_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def on_imu_data_received(self, data):
        self.imu_data = data
        self.new_imu_data = True

    def calculate_euler_from_imu(self):
        quat = (
            self.imu_data.orientation.x,
            self.imu_data.orientation.y,
            self.imu_data.orientation.z,
            self.imu_data.orientation.w,
        )
        self.roll, self.pitch, self.yaw = transformations.euler_from_quaternion(quat)
        self.last_heading = (self.euler_interpolator(self.yaw) + MappingSettings.DECLINATION_OFFSET) % 360

    def __mouse_released_event(self, event):
        self.x_drag_start = -1
        self.y_drag_start = -1


    def __mouse_wheel_event(self, event):
        # print "wheel:", event.angleDelta()
        self.zoom_subtraction += event.angleDelta().y() / 12

    def __mouse_move_event(self, event):
        if self.x_drag_start != -1 and self.x_drag_start != -1:
            buttons = event.buttons()

            if buttons == QtCore.Qt.LeftButton:
                x_pos = event.pos().x()
                y_pos = event.pos().y()
                dx = x_pos - self.x_drag_start
                dy = y_pos - self.y_drag_start

                self.x_drag = dx
                self.y_drag = dy
        else:
            self.x_drag_start = event.pos().x()
            self.y_drag_start = event.pos().y()

    @staticmethod
    def constrain(val, min_val, max_val):
        return min(max_val, max(min_val, val))