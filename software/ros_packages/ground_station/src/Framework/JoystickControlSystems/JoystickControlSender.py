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

Y_AXIS_DEADBAND = 0.05
X_AXIS_DEADBAND = 0.05

THROTTLE_MIN = 0.05

CAMERA_CHANGE_TIME = 0.25
GUI_ELEMENT_CHANGE_TIME = 0.25


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
            "x_axis": 512,
            "y_axis": 512,
            "z_axis": 128,
            "throttle_axis": 128,

            "hat_x_axis": 0,
            "hat_y_axis": 0,

            "trigger_pressed": 0,
            "thumb_pressed": 0,
            "three_pressed": 0,
            "four_pressed": 0,
            "five_pressed": 0,
            "six_pressed": 0,

            "eleven_pressed": 0,
            "twelve_pressed": 0,
            "thirteen_pressed": 0,
            "fourteen_pressed": 0,
            "fifteen_pressed": 0,
            "sixteen_pressed": 0,
        }

        self.raw_mapping_to_class_mapping = {
            "ABS_X": "x_axis",
            "ABS_Y": "y_axis",
            "ABS_RZ": "z_axis",
            "ABS_THROTTLE": "throttle_axis",

            "ABS_HAT0X": "hat_x_axis",
            "ABS_HAT0Y": "hat_y_axis",

            "BTN_TRIGGER": "trigger_pressed",
            "BTN_THUMB": "thumb_pressed",
            "BTN_THUMB2": "three_pressed",
            "BTN_TOP": "four_pressed",
            "BTN_TOP2": "five_pressed",
            "BTN_PINKIE": "six_pressed",

            "BTN_BASE5": "eleven_pressed",
            "BTN_BASE6": "twelve_pressed",
            "BTN_BASE3": "thirteen_pressed",
            "BTN_BASE4": "fourteen_pressed",
            "BTN_BASE": "fifteen_pressed",
            "BTN_BASE2": "sixteen_pressed",
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


#####################################
# Controller Class Definition
#####################################
class JoystickControlSender(QtCore.QThread):
    set_speed_limit__signal = QtCore.pyqtSignal(int)
    set_left_drive_output__signal = QtCore.pyqtSignal(int)
    set_right_drive_output__signal = QtCore.pyqtSignal(int)

    change_gui_element_selection__signal = QtCore.pyqtSignal(int)
    change_camera_selection__signal = QtCore.pyqtSignal(int)

    def __init__(self, shared_objects):
        super(JoystickControlSender, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]
        self.speed_limit_progress_bar = self.right_screen.speed_limit_progress_bar  # type: QtWidgets.QProgressBar
        self.left_drive_progress_bar = self.right_screen.left_drive_progress_bar  # type: QtWidgets.QProgressBar
        self.right_drive_progress_bar = self.right_screen.right_drive_progress_bar  # type: QtWidgets.QProgressBar

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

        self.wait_time = 1.0 / DRIVE_COMMAND_HERTZ

        self.drive_paused = True

        self.last_gui_element_change_time = time()
        self.last_camera_change_time = time()

    def run(self):
        while self.run_thread_flag:

            start_time = time()

            self.check_and_set_pause_state()
            self.__update_and_publish()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def connect_signals_and_slots(self):
        self.set_speed_limit__signal.connect(self.speed_limit_progress_bar.setValue)
        self.set_left_drive_output__signal.connect(self.left_drive_progress_bar.setValue)
        self.set_right_drive_output__signal.connect(self.right_drive_progress_bar.setValue)

    def check_and_set_pause_state(self):
        if self.joystick.controller_states["thumb_pressed"]:
            self.drive_paused = not self.drive_paused
            self.msleep(350)
            self.show_changed_pause_state()

    def __update_and_publish(self):
        self.publish_drive_command()
        self.publish_camera_control_commands()

    def publish_drive_command(self):
        throttle_axis = max((255 - self.joystick.controller_states["throttle_axis"]) / 255.0, THROTTLE_MIN)

        if self.drive_paused:
            drive_message = DriveCommandMessage()
        else:
            drive_message = self.get_drive_message(throttle_axis)

        left_output = abs(drive_message.drive_twist.linear.x - drive_message.drive_twist.angular.z)
        right_output = abs(drive_message.drive_twist.linear.x + drive_message.drive_twist.angular.z)

        self.set_speed_limit__signal.emit(throttle_axis * 100)
        self.set_left_drive_output__signal.emit(left_output * 100)
        self.set_right_drive_output__signal.emit(right_output * 100)

        self.drive_command_publisher.publish(drive_message)

    def publish_camera_control_commands(self):
        three_pressed = self.joystick.controller_states["three_pressed"]
        four_pressed = self.joystick.controller_states["four_pressed"]
        five_pressed = self.joystick.controller_states["five_pressed"]
        six_pressed = self.joystick.controller_states["six_pressed"]

        if (five_pressed or six_pressed) and (time() - self.last_camera_change_time) > CAMERA_CHANGE_TIME:
            change = -1 if five_pressed else 1
            self.change_camera_selection__signal.emit(change)
            self.last_camera_change_time = time()

        if (three_pressed or four_pressed) and (time() - self.last_gui_element_change_time) > GUI_ELEMENT_CHANGE_TIME:
            change = -1 if three_pressed else 1
            self.change_gui_element_selection__signal.emit(change)
            self.last_gui_element_change_time = time()

    def get_drive_message(self, throttle_axis):
        drive_message = DriveCommandMessage()

        y_axis = throttle_axis * (-(self.joystick.controller_states["y_axis"] - 512) / 512.0)
        z_axis = throttle_axis * (-(self.joystick.controller_states["z_axis"] - 128) / 128.0)
        x_axis = throttle_axis * (-(self.joystick.controller_states["x_axis"] - 512) / 512.0)

        if abs(y_axis) < Y_AXIS_DEADBAND:
            y_axis = 0

        if abs(x_axis) < X_AXIS_DEADBAND and y_axis == 0:
            twist = z_axis
        else:
            twist = x_axis if y_axis >= 0 else -x_axis

        drive_message.drive_twist.linear.x = y_axis
        drive_message.drive_twist.angular.z = twist

        return drive_message

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def show_changed_pause_state(self):
        if self.drive_paused:
            self.left_drive_progress_bar.setStyleSheet("background-color:darkred;")
            self.right_drive_progress_bar.setStyleSheet("background-color:darkred;")
        else:
            self.left_drive_progress_bar.setStyleSheet("background-color: transparent;")
            self.right_drive_progress_bar.setStyleSheet("background-color: transparent;")

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
