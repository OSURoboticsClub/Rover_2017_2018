#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from rover_control.msg import DriveCommandMessage

#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Logitech Logitech Extreme 3D Pro"

DEFAULT_DRIVE_COMMAND_TOPIC = "/rover_control/command_control/ground_station_drive"

DRIVE_COMMAND_HERTZ = 15


#####################################
# Controller Class Definition
#####################################
class LogitechJoystick(QtCore.QThread):
    def __init__(self):
        super(LogitechJoystick, self).__init__()

        # ########## Thread Flags ##########
        self.run_thread_flag = True
        self.setup_controller_flag = True
        self.data_acquisition_and_broadcast_flag = True
        self.controller_acquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "left_stick_x_axis": 0,
            "y_axis": 512,
            "left_stick_center_pressed": 0,

            "right_stick_x_axis": 0,
            "right_stick_y_axis": 0,
            "right_stick_center_pressed": 0,

            "left_trigger_z_axis": 0,
            "left_bumper_pressed": 0,

            "z_axis": 128,
            "right_bumper_pressed": 0,

            "dpad_x": 0,
            "dpad_y": 0,

            "select_pressed": 0,
            "start_pressed": 0,
            "home_pressed": 0,

            "a_pressed": 0,
            "b_pressed": 0,
            "x_pressed": 0,
            "y_pressed": 0
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "left_stick_x_axis",
            "ABS_Y": "y_axis",
            "BTN_THUMBL": "left_stick_center_pressed",

            "ABS_RX": "right_stick_x_axis",
            "ABS_RY": "right_stick_y_axis",
            "BTN_THUMBR": "right_stick_center_pressed",

            "ABS_Z": "left_trigger_z_axis",
            "BTN_TL": "left_bumper_pressed",

            "ABS_RZ": "z_axis",
            "BTN_TR": "right_bumper_pressed",

            "ABS_HAT0X": "dpad_x",
            "ABS_HAT0Y": "dpad_y",

            "BTN_SELECT": "select_pressed",
            "BTN_START": "start_pressed",
            "BTN_MODE": "home_pressed",

            "BTN_SOUTH": "a_pressed",
            "BTN_EAST": "b_pressed",
            "BTN_NORTH": "x_pressed",
            "BTN_WEST": "y_pressed"
        }

        self.ready = False

        self.start()

    def run(self):

        while self.run_thread_flag:
            if self.setup_controller_flag:
                self.controller_acquired = self.__setup_controller()
                self.setup_controller_flag = False
            if self.data_acquisition_and_broadcast_flag:
                self.__get_controller_data()

    def __setup_controller(self):
        for device in devices.gamepads:
            print device
            if device.name == GAME_CONTROLLER_NAME:
                self.gamepad = device

                return True
        return False

    def __get_controller_data(self):
        if self.controller_acquired:
            events = self.gamepad.read()

            for event in events:
                if event.code in self.raw_mapping_to_class_mapping:
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state
            self.ready = True
            # print "Logitech: %s" % self.controller_states


#####################################
# Controller Class Definition
#####################################
class RoverDriveSender(QtCore.QThread):
    def __init__(self, shared_objects):
        super(RoverDriveSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.primary_video_display_label = self.right_screen.primary_video_label  # type:QtWidgets.QLabel
        self.secondary_video_display_label = self.right_screen.secondary_video_label  # type:QtWidgets.QLabel
        self.tertiary_video_display_label = self.right_screen.tertiary_video_label  # type:QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.joystick = LogitechJoystick()

        # ########## Class Variables ##########
        # Publishers
        self.drive_command_publisher = rospy.Publisher(DEFAULT_DRIVE_COMMAND_TOPIC, DriveCommandMessage, queue_size=1)

        self.wait_time = 1 / DRIVE_COMMAND_HERTZ

    def run(self):
        while self.run_thread_flag:

            start_time = time()

            self.__update_and_publish()

            time_diff = time() - start_time

            self.msleep(max(self.wait_time - time_diff, 0))

    def connect_signals_and_slots(self):
        pass

    def __update_and_publish(self):
        drive_message = DriveCommandMessage()

        drive_message.drive_twist.linear.x = -(self.joystick.controller_states["y_axis"] - 512) / 1024.0
        drive_message.drive_twist.angular.z = -(self.joystick.controller_states["z_axis"] - 128) / 255.0
        self.drive_command_publisher.publish(drive_message)
        # print self.joystick.controller_states["y_axis"], self.joystick.controller_states["z_axis"]

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
