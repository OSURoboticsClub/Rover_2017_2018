#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
from inputs import devices, GamePad
from time import time

import rospy
from rover_arm.msg import ArmControlMessage

#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Afterglow Gamepad for Xbox 360"

DRIVE_COMMAND_HERTZ = 20

RELATIVE_ARM_CONTROL_TOPIC = "/rover_arm/control/relative"

BASE_SCALAR = 0.003
SHOULDER_SCALAR = 0.002
ELBOW_SCALAR = 0.002
ROLL_SCALAR = 0.003
WRIST_PITCH_SCALAR = 0.003
WRIST_ROLL_SCALAR = 0.006

LEFT_X_AXIS_DEADZONE = 1500
LEFT_Y_AXIS_DEADZONE = 1500

RIGHT_X_AXIS_DEADZONE = 1500
RIGHT_Y_AXIS_DEADZONE = 1500


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

        self.relative_arm_control_publisher = rospy.Publisher(RELATIVE_ARM_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)

    def run(self):
        self.logger.debug("Starting Joystick Thread")

        while self.run_thread_flag:
            start_time = time()

            self.process_and_send_arm_control()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping Joystick Thread")

    def connect_signals_and_slots(self):
        pass

    def process_and_send_arm_control(self):
        message = ArmControlMessage()

        should_publish = False

        left_trigger = self.controller.controller_states["left_trigger"]
        right_trigger = self.controller.controller_states["right_trigger"]

        left_x_axis = self.controller.controller_states["left_x_axis"] if abs(self.controller.controller_states[
                                                                              "left_x_axis"]) > LEFT_X_AXIS_DEADZONE else 0
        left_y_axis = self.controller.controller_states["left_y_axis"] if abs(self.controller.controller_states[
                                                                              "left_y_axis"]) > LEFT_Y_AXIS_DEADZONE else 0
        right_y_axis = self.controller.controller_states["right_y_axis"] if abs(self.controller.controller_states[
                                                                              "right_y_axis"]) > RIGHT_Y_AXIS_DEADZONE else 0
        right_x_axis = self.controller.controller_states["right_x_axis"] if abs(self.controller.controller_states[
                                                                              "right_x_axis"]) > RIGHT_X_AXIS_DEADZONE else 0

        # print left_x_axis, ":", left_y_axis, ":", right_x_axis, ":", right_y_axis

        left_trigger_ratio = left_trigger / 255.0
        right_trigger_ratio = right_trigger / 255.0

        if left_trigger > 25:
            should_publish = True
            message.base = ((left_x_axis / 32768.0) * BASE_SCALAR) * left_trigger_ratio
            message.shoulder = (-(left_y_axis / 32768.0) * SHOULDER_SCALAR) * left_trigger_ratio
            message.elbow = ((right_y_axis / 32768.0) * ELBOW_SCALAR) * left_trigger_ratio
            message.roll = (-(right_x_axis / 32768.0) * ROLL_SCALAR) * left_trigger_ratio

        elif right_trigger > 25:
            should_publish = True
            message.wrist_roll = ((right_x_axis / 32768.0) * WRIST_ROLL_SCALAR) * right_trigger_ratio
            message.wrist_pitch = ((left_y_axis / 32768.0) * WRIST_PITCH_SCALAR) * right_trigger_ratio

        if should_publish:
            self.relative_arm_control_publisher.publish(message)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
