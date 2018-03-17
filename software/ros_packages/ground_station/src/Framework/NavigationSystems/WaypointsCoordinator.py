from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy


class WaypointsCoordinator(QtCore.QThread):
    new_manual_waypoint_entry = QtCore.pyqtSignal(str, float, float, int)
    update_waypoint_entry = QtCore.pyqtSignal(str, str, str, int)

    def __init__(self, shared_objects):
        super(WaypointsCoordinator, self).__init__()
        self.run_thread_flag = True

        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.navigation_label = (self.left_screen.
                                 navigation_waypoints_table_widget)
        self.landmark_label = self.left_screen.landmark_waypoints_table_widget

        self.name_edit_label = (self.left_screen.
                                manual_waypoint_landmark_name_line_edit)
        self.latitude_label = (self.left_screen.
                               manual_waypoint_decimal_lattitude_spin_box)
        self.longitude_label = (self.left_screen.
                                manual_waypoint_decimal_longitude_spin_box)

        self.settings = QtCore.QSettings()

        self.logger = logging.getLogger("groundstation")

    def run(self):
        while self.run_thread_flag:
            self.msleep(3)

    def connect_signals_and_slots(self):
        self.new_manual_waypoint_entry.connect(self.update_manual_entry)

        # setting up signals to save for Navigation Table
        # self.

        self.navigation_label.cellClicked.connect(self._on_nav_clicked)
        self.landmark_label.cellClicked.connect(self._on_land_clicked)

    def setup_signals(self, start_signal, 
                      signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested_slot)
    
    def on_kill_threads_requested_slot(self):
        self.run_thread_flag = False

    def update_manual_entry(self, name, lat, lng, table):
        print name, lat, lng, table
        self.name_edit_label.setReadOnly(table+1 % 2)
        self.name_edit_label.setText(name)
        self.latitude_label.setValue(lat)
        self.longitude_label.setValue(lng)

    def _on_nav_clicked(self, row, col):
        print "nav" + str(row), str(col)
        self.new_manual_waypoint_entry.emit(
            self.navigation_label.item(row, 0).text(),
            float(self.navigation_label.item(row, 1).text()),
            float(self.navigation_label.item(row, 2).text()),
            0
        )

    def _on_land_clicked(self, row, col):
        print "land" + str(row), str(col)
        self.new_manual_waypoint_entry.emit(
            self.landmark_label.item(row, 0).text(),
            float(self.landmark_label.item(row, 1).text()),
            float(self.landmark_label.item(row, 2).text()),
            1
        )