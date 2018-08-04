#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from rover_control.msg import DriveCommandMessage, TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Afterglow Gamepad for Xbox 360"

DRIVE_COMMAND_HERTZ = 20


#####################################
# Controller Class Definition
#####################################
class XBOXController(QtCore.QThread):
    def __init__(self):
        super(XBOXController, self).__init__()

        # ########## Thread Flags ##########
        self.run_thread_flag = True
        self.setup_controller_flag = True
        self.data_acquisition_and_broadcast_flag = True
        self.controller_acquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "left_x_axis": 0,
            "left_y_axis": 0,
            "left_stick_button": 0,

            "right_x_axis": 0,
            "right_y_axis": 0,
            "right_stick_button": 0,

            "left_trigger": 0,
            "left_bumper": 0,

            "right_trigger": 0,
            "right_bumper": 0,

            "hat_x_axis": 0,
            "hat_y_axis": 0,

            "back_button": 0,
            "start_button": 0,
            "xbox_button": 0,

            "x_button": 0,
            "a_button": 0,
            "b_button": 0,
            "y_button": 0
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "left_x_axis",
            "ABS_Y": "left_y_axis",
            "BTN_THUMBL": "left_stick_button",

            "ABS_RX": "right_x_axis",
            "ABS_RY": "right_y_axis",
            "BTN_THUMBR": "right_stick_button",

            "ABS_Z": "left_trigger",
            "BTN_TL": "left_bumper",

            "ABS_RZ": "right_trigger",
            "BTN_TR": "right_bumper",

            "ABS_HAT0X": "hat_x_axis",
            "ABS_HAT0Y": "hat_y_axis",

            "BTN_SELECT": "back_button",
            "BTN_START": "start_button",
            "BTN_MODE": "xbox_button",

            "BTN_NORTH": "x_button",
            "BTN_SOUTH": "a_button",
            "BTN_EAST": "b_button",
            "BTN_WEST": "y_button"
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
                # For seeing codes you haven't added yet...
                # if event.code not in self.raw_mapping_to_class_mapping and event.code != "SYN_REPORT":
                #     print event.code, ":", event.state

                if event.code in self.raw_mapping_to_class_mapping:
                    # print event.code, ":", event.state
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state

            self.ready = True


#####################################
# Controller Class Definition
#####################################
class ControllerControlSender(QtCore.QThread):

    def __init__(self, shared_objects):
        super(ControllerControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.video_coordinator = self.shared_objects["threaded_classes"]["Video Coordinator"]
        self.right_screen = self.shared_objects["screens"]["right_screen"]

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        self.controller = XBOXController()

        # ########## Class Variables ##########

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

    def run(self):
        self.logger.debug("Starting Joystick Thread")

        while self.run_thread_flag:
            start_time = time()

            # print self.controller.controller_states

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping Joystick Thread")

    def connect_signals_and_slots(self):
        pass

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
