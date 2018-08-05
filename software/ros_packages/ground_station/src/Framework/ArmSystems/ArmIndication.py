# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy

from rover_arm.msg import ArmStatusMessage

#####################################
# Global Variables
#####################################
ARM_STATUS_TOPIC = "/rover_arm/status"


#####################################
# Controller Class Definition
#####################################
class ArmIndication(QtCore.QObject):
    base_position_updated__signal = QtCore.pyqtSignal(int)
    shoulder_position_updated__signal = QtCore.pyqtSignal(int)
    elbow_position_updated__signal = QtCore.pyqtSignal(int)
    roll_position_updated__signal = QtCore.pyqtSignal(int)
    wrist_pitch_position_updated__signal = QtCore.pyqtSignal(int)
    wrist_roll_position_updated__signal = QtCore.pyqtSignal(int)

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

        # ########## Class Variables ##########
        self.arm_status_subscriber = rospy.Subscriber(ARM_STATUS_TOPIC, ArmStatusMessage, self.on_arm_status_update_received__callback)

        # ########## Connect Signals and Slots ##########
        self.connect_signals_and_slots()

    def connect_signals_and_slots(self):
        self.base_position_updated__signal.connect(self.base_rotation_dial.setValue)
        self.shoulder_position_updated__signal.connect(self.shoulder_pitch_dial.setValue)
        self.elbow_position_updated__signal.connect(self.elbow_pitch_dial.setValue)
        self.roll_position_updated__signal.connect(self.elbow_roll_dial.setValue)
        self.wrist_pitch_position_updated__signal.connect(self.end_effector_pitch_dial.setValue)
        self.wrist_roll_position_updated__signal.connect(self.end_effector_rotation_dial.setValue)

    def on_arm_status_update_received__callback(self, data):
        self.base_position_updated__signal.emit(int(data.base * 1000))
        self.shoulder_position_updated__signal.emit(int(data.shoulder * 1000))
        self.elbow_position_updated__signal.emit(int(data.elbow * 1000))
        self.roll_position_updated__signal.emit(int(data.roll * 1000))
        self.wrist_pitch_position_updated__signal.emit(int(data.wrist_pitch * 1000))
        self.wrist_roll_position_updated__signal.emit(int(data.wrist_roll * 1000))


