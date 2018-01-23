#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy

# Custom Imports
import Framework.VideoSystems.RoverVideoCoordinator as RoverVideoCoordinator

#####################################
# Global Variables
#####################################
UI_FILE_LEFT = "Resources/Ui/left_screen.ui"
UI_FILE_RIGHT = "Resources/Ui/right_screen.ui"

#####################################
# Class Organization
#####################################
# Class Name:
#   "init"
#   "run (if there)" - personal pref
#   "private methods"
#   "public methods, minus slots"
#   "slot methods"
#   "static methods"
#   "run (if there)" - personal pref


#####################################
# ApplicationWindow Class Definition
#####################################
class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None, ui_file_path=None):
        super(ApplicationWindow, self).__init__(parent)

        uic.loadUi(ui_file_path, self)

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


#####################################
# GroundStation Class Definition
#####################################
class GroundStation(QtCore.QObject):
    LEFT_SCREEN_ID = 0
    RIGHT_SCREEN_ID = 1

    start_threads_signal = QtCore.pyqtSignal()
    connect_signals_and_slots_signal = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        rospy.init_node("ground_station")

        self.shared_objects = {
            "screens": {},
            "regular_classes": {},
            "threaded_classes": {}
        }

        # ###### Instantiate Left And Right Screens #####
        self.shared_objects["screens"]["left_screen"] = \
            self.create_application_window(UI_FILE_LEFT, "Rover Ground Station Left Screen",
                                           self.LEFT_SCREEN_ID)  # type: ApplicationWindow

        self.shared_objects["screens"]["right_screen"] = \
            self.create_application_window(UI_FILE_RIGHT, "Rover Ground Station Right Screen",
                                           self.RIGHT_SCREEN_ID)  # type: ApplicationWindow

        # ##### Instantiate Simple Classes #####

        # ##### Instantiate Threaded Classes #####
        self
        # self.__add_thread("Primary Video",
        #                   RoverVideoReceiver.RoverVideoReceiver(
        #                       self.shared_objects,
        #                       self.shared_objects["screens"]["right_screen"].primary_video_label,
        #                       "/cameras/main_navigation/"))
        # self.__add_thread("Secondary Video",
        #                   RoverVideoReceiverOld.VideoTest(
        #                       self.shared_objects,
        #                       self.shared_objects["screens"]["right_screen"].secondary_video_label,
        #                       (640, 360),
        #                       sub_path="/cameras/chassis/image_640x360/compressed"))
        # self.__add_thread("Tertiary Video",
        #                   RoverVideoReceiverOld.VideoTest(
        #                       self.shared_objects,
        #                       self.shared_objects["screens"]["right_screen"].tertiary_video_label,
        #                       (640, 360),
        #                       sub_path="/cameras/undercarriage/image_640x360/compressed"))

        self.connect_signals_and_slots_signal.emit()
        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def __add_thread(self, thread_name, instance):
        self.shared_objects["threaded_classes"][thread_name] = instance
        instance.setup_signals(self.start_threads_signal, self.connect_signals_and_slots_signal, self.kill_threads_signal)

    def __connect_signals_to_slots(self):
        self.shared_objects["screens"]["left_screen"].exit_requested_signal.connect(self.on_exit_requested__slot)
        self.shared_objects["screens"]["right_screen"].exit_requested_signal.connect(self.on_exit_requested__slot)

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        # Wait for Threads
        for thread in self.threads["threaded_classes"]:
            self.threads["threaded_classes"][thread].wait()

        QtGui.QGuiApplication.exit()

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        app_window = ApplicationWindow(parent=None, ui_file_path=ui_file_path)  # Make a window in this application
        app_window.setWindowTitle(title)  # Sets the window title

        app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
                                  QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
                                  QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
                                  QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        app_window.showFullScreen()  # Shows the window in full screen mode

        return app_window


#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # This allows the keyboard interrupt kill to work properly

    application = QtWidgets.QApplication(sys.argv)  # Create the ase qt gui application

    ground_station = GroundStation()

    application.exec_()  # Execute launching of the application
