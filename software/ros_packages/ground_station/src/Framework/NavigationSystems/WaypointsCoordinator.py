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

        self.navigation_label = self.left_screen.tableWidget
        self.landmark_label = self.left_screen.tableWidget_2

        # to get access to the values, iteration is needed.
        self.lat_long_grid = self.left_screen.gridLayout_7

        self.settings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

    def connect_signals_and_slots(self):
        self.new_manual_waypoint_entry.connect(self.update_manual_entry)

        self.navigation_label.cellClicked.connect(self._on_nav_clicked)
        self.landmark_label.cellClicked.connect(self.__on_land_clicked)

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
