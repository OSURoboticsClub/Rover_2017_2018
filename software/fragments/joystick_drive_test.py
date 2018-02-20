"""
    This file contains the live logs page sub-class
"""

#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from inputs import devices, GamePad
import sys
import time

import rospy
from rover_drive.msg import RoverMotorDrive


#####################################
# Global Variables
#####################################
GAME_CONTROLLER_NAME = "Logitech Logitech Extreme 3D Pro"
CONTROLLER_DATA_UPDATE_FREQUENCY = 100  # Times per second


#####################################
# Controller Class Definition
#####################################

# ########## Signals ##########
controller_connection_aquired = QtCore.pyqtSignal(bool)
controller_update_ready_signal = QtCore.pyqtSignal()


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
        self.controller_aquired = False

        # ########## Class Variables ##########
        self.gamepad = None  # type: GamePad

        self.controller_states = {
            "left_stick_x_axis": 0,
            "left_stick_y_axis": 0,
            "left_stick_center_pressed": 0,

            "right_stick_x_axis": 0,
            "right_stick_y_axis": 0,
            "right_stick_center_pressed": 0,

            "left_trigger_z_axis": 0,
            "left_bumper_pressed": 0,

            "right_trigger_z_axis": 0,
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
            "ABS_Y": "left_stick_y_axis",
            "BTN_THUMBL": "left_stick_center_pressed",

            "ABS_RX": "right_stick_x_axis",
            "ABS_RY": "right_stick_y_axis",
            "BTN_THUMBR": "right_stick_center_pressed",

            "ABS_Z": "left_trigger_z_axis",
            "BTN_TL": "left_bumper_pressed",

            "ABS_RZ": "right_trigger_z_axis",
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

        self.last_time = time.time()

        rospy.init_node("drive_tester")

        self.pub = rospy.Publisher("/drive/motoroneandtwo", RoverMotorDrive, queue_size=1)

        self.start()


    def run(self):

        while self.run_thread_flag:
            if self.setup_controller_flag:
                self.controller_aquired = self.__setup_controller()
                self.setup_controller_flag = False
            if self.data_acquisition_and_broadcast_flag:
                self.__get_controller_data()
                self.__broadcast_if_ready()

            # self.msleep(100)


    # noinspection PyUnresolvedReferences
    def connect_signals_to_slots__slot(self):
        pass


    def __setup_controller(self):
        for device in devices.gamepads:
            if device.name == GAME_CONTROLLER_NAME:
                self.gamepad = device
                return True
        return False


    def __get_controller_data(self):
        if (self.controller_aquired):
            events = self.gamepad.read()

            for event in events:
                if event.code in self.raw_mapping_to_class_mapping:
                    self.controller_states[self.raw_mapping_to_class_mapping[event.code]] = event.state
                    # print "Logitech: %s" % self.controller_states


    def __broadcast_if_ready(self):
        current_time = time.time()

        if (current_time - self.last_time) > (1 / CONTROLLER_DATA_UPDATE_FREQUENCY):
            drive = RoverMotorDrive()

            axis = self.controller_states["left_stick_y_axis"]

            drive.first_motor_direction = 1 if axis <= 512 else 0
            drive.first_motor_speed = min(abs(self.controller_states["left_stick_y_axis"] - 512) * 128, 65535)

            self.pub.publish(drive)
            self.last_time = current_time



    def on_kill_threads__slot(self):
        self.terminate()  # DON'T normally do this!!!!!
        self.run_thread_flag = False


if __name__ == '__main__':
    qapp = QtCore.QCoreApplication(sys.argv)

    joystick = LogitechJoystick()

    qapp.exec_()



