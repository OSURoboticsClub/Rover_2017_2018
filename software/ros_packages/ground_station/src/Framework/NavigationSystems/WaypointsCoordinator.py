from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy


class WaypointsCoordinator(QtCore.QThread):
    new_manual_waypoint_entry = QtCore.pyqtSignal(str, str, str, int)
    update_waypoint_entry = QtCore.pyqtSignal(str, str, int)

    def __init__(self, shared_objects):
        super(WaypointsCoordinator, self).init()

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.navigation_label = self.left_screen.navigation_waypoints_table_widget
        self.landmark_label = self.left_screen.landmark_waypoints_table_widget

        self.name_edit_label = (self.left_screen.
                                manual_waypoint_landmark_name_line_edit)
        self.latitude_label = (self.left_screen.
                               manual_waypoint_decimal_lattitude_spin_box)
        self.longitude_label = (self.left_screen.
                                manual_waypoint_decimal_longitude_spin_box)

        self.settings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

    def connect_signals_and_slots(self):
        self.new_manual_waypoint_entry.connect(self.update_manual_entry)

        # setting up signals to save for Navigation Table
        # self.

        self.navigation_label.cellClicked.connect(self._on_nav_clicked)
        self.landmark_label.cellClicked.connect(self.__on_land_clicked)

    def update_manual_entry(self, name, lat, lng, table):
        if table == 1:
            self.name_edit_label.setText(name)
        self.latitude_label.setText(lat)
        self.longitude_label.set(lng)

    def _on_nav_clicked(self, row, col):
        self.update_waypoint_entry.emit(
            self.navigation_label.item(row, 0),
            self.navigation_label.item(row, 1),
            self.navigation_label.item(row, 2),
            0
        )

    def _on_land_clicked(self, row, col):
        self.update_waypoint_entry.emit(
            self.landmark_label.item(row, 0),
            self.landmark_label.item(row, 1),
            self.landmark_label.item(row, 2),
            1
        )
