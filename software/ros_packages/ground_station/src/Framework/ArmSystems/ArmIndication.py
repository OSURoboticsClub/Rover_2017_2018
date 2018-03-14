# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
from time import time

#####################################
# Global Variables
#####################################
THREAD_HERTZ = 2

ROTATION_SPEED_MODIFIER = 0.15


#####################################
# Controller Class Definition
#####################################
class ArmIndication(QtCore.QThread):
    arm_joint_position_updated__signal = QtCore.pyqtSignal(int)

    def __init__(self, shared_objects):
        super(ArmIndication, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.right_screen = self.shared_objects["screens"]["right_screen"]

        self.base_rotation_dial = self.right_screen.base_rotation_dial  # type: QtWidgets.QDial
        self.shoulder_pitch_dial = self.right_screen.shoulder_pitch_dial  # type: QtWidgets.QDial
        self.elbow_pitch_dial = self.right_screen.elbow_pitch_dial  # type: QtWidgets.QDial
        self.elbow_roll_dial = self.right_screen.elbow_roll_dial  # type: QtWidgets.QDial
        self.end_effector_pitch_dial = self.right_screen.end_effector_pitch_dial  # type: QtWidgets.QDial
        self.end_effector_rotation_dial = self.right_screen.end_effector_rotation_dial  # type: QtWidgets.QDial

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.current_position_delta = 0
        self.shown_position = 0

    def run(self):
        while self.run_thread_flag:
            start_time = time()

            self.change_position_if_needed()

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

    def change_position_if_needed(self):
        self.shown_position += self.current_position_delta * ROTATION_SPEED_MODIFIER
        self.shown_position %= 360

        self.arm_joint_position_updated__signal.emit(self.shown_position)

    def __on_position_change_requested__slot(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.current_position_delta = 1
        elif event.button() == QtCore.Qt.RightButton:
            self.current_position_delta = 0

    def connect_signals_and_slots(self):
        self.arm_joint_position_updated__signal.connect(self.base_rotation_dial.setValue)
        self.arm_joint_position_updated__signal.connect(self.shoulder_pitch_dial.setValue)
        self.arm_joint_position_updated__signal.connect(self.elbow_pitch_dial.setValue)
        self.arm_joint_position_updated__signal.connect(self.elbow_roll_dial.setValue)
        self.arm_joint_position_updated__signal.connect(self.end_effector_pitch_dial.setValue)
        self.arm_joint_position_updated__signal.connect(self.end_effector_rotation_dial.setValue)

        self.base_rotation_dial.mousePressEvent = self.__on_position_change_requested__slot
        self.shoulder_pitch_dial.mousePressEvent = self.__on_position_change_requested__slot
        self.elbow_pitch_dial.mousePressEvent = self.__on_position_change_requested__slot
        self.elbow_roll_dial.mousePressEvent = self.__on_position_change_requested__slot
        self.end_effector_pitch_dial.mousePressEvent = self.__on_position_change_requested__slot
        self.end_effector_rotation_dial.mousePressEvent = self.__on_position_change_requested__slot

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
