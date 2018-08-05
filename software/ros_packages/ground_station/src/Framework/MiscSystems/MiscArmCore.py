# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy
from time import time

from rover_arm.msg import ArmControlMessage, ArmStatusMessage

#####################################
# Global Variables
#####################################
ARM_RELATIVE_CONTROL_TOPIC = "/rover_arm/control/relative"
ARM_ABSOLUTE_CONTROL_TOPIC = "/rover_arm/control/absolute"
ARM_STATUS_TOPIC = "/rover_arm/status"

THREAD_HERTZ = 5

COMMS_TO_STRING = {
    0: "NO STATUS",
    1: "COMMS OK",
    2: "NO DEVICE",
    4: "BUS ERROR",
    8: "GEN COMM ERROR",
    16: "PARAMETER ERROR",
    32: "LENGTH ERROR"
}

TARGET_REACHED_BIT_POSITION = 1

STATUS_TO_STRING = {
    1: "TARGET REACHED",
    2: "ERROR RECOVERY",
    3: "RUN",
    4: "ENABLED",
    5: "FAULT STOP",
    6: "WARNING",
    7: "STO ACTIVE",
    8: "SERVO READY",
    10: "BRAKING",
    11: "HOMING",
    12: "INITIALIZED",
    13: "VOLT OK",
    15: "PERMANENT STOP"
}

FAULT_TO_STRING = {
    1: "TRACKING ERROR",
    2: "OVER CURRENT",
    # 3: "COMMUNICATION ERROR",  # Was showing even though things were working???
    4: "ENCODER FAILURE",
    5: "OVER TEMP",
    6: "UNDER VOLTAGE",
    7: "OVER VOLTAGE",
    8: "PROG OR MEM ERROR",
    9: "HARDWARE ERROR",
    10: "OVER VELOCITY",
    11: "INIT ERROR",
    12: "MOTION ERROR",
    13: "RANGE ERROR",
    14: "POWER STAGE FORCED OFF",
    15: "HOST COMM ERROR"
}

POSITIONAL_TOLERANCE = 0.02

# Order is [base, shoulder, elbow, roll, wrist_pitch, wrist_roll]
ARM_STOW_PROCEDURE = [
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0],
    [0.0, -0.035, -0.28, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.5, -0.25, 0.25, 0.0],
    [0.0, -0.25, -0.5, -0.25, 0.25, -0.25]
]

ARM_UNSTOW_PROCEDURE = [
    [0.0, -0.25, -0.5, -0.25, 0.25, -0.25],
    [0.0, -0.035, -0.5, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.28, -0.25, 0.25, 0.0],
    [0.0, -0.035, -0.28, 0.0, 0.0, 0.0]
]


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class MiscArm(QtCore.QThread):
    base_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    roll_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_comms_state_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_comms_state_update_ready__signal = QtCore.pyqtSignal(str)

    base_status_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_status_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_status_update_ready__signal = QtCore.pyqtSignal(str)
    roll_status_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_status_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_status_update_ready__signal = QtCore.pyqtSignal(str)

    base_faults_update_ready__signal = QtCore.pyqtSignal(str)
    shoulder_faults_update_ready__signal = QtCore.pyqtSignal(str)
    elbow_faults_update_ready__signal = QtCore.pyqtSignal(str)
    roll_faults_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_pitch_faults_update_ready__signal = QtCore.pyqtSignal(str)
    wrist_roll_faults_update_ready__signal = QtCore.pyqtSignal(str)

    def __init__(self, shared_objects):
        super(MiscArm, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.arm_control_upright_zeroed_button = self.left_screen.arm_control_upright_zeroed_button  # type:QtWidgets.QPushButton
        self.arm_controls_stow_arm_button = self.left_screen.arm_controls_stow_arm_button  # type:QtWidgets.QPushButton
        self.arm_controls_unstow_arm_button = self.left_screen.arm_controls_unstow_arm_button  # type:QtWidgets.QPushButton

        self.arm_controls_calibration_button = self.left_screen.arm_controls_calibration_button  # type:QtWidgets.QPushButton
        self.arm_controls_clear_faults_button = self.left_screen.arm_controls_clear_faults_button  # type:QtWidgets.QPushButton
        self.arm_controls_reset_motor_drivers_button = self.left_screen.arm_controls_reset_motor_drivers_button  # type:QtWidgets.QPushButton

        self.arm_controls_base_comms_label = self.left_screen.arm_controls_base_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_base_status_label = self.left_screen.arm_controls_base_status_label  # type:QtWidgets.QLabel
        self.arm_controls_base_faults_label = self.left_screen.arm_controls_base_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_shoulder_comms_label = self.left_screen.arm_controls_shoulder_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_shoulder_status_label = self.left_screen.arm_controls_shoulder_status_label  # type:QtWidgets.QLabel
        self.arm_controls_shoulder_faults_label = self.left_screen.arm_controls_shoulder_faults_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_comms_label = self.left_screen.arm_controls_elbow_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_status_label = self.left_screen.arm_controls_elbow_status_label  # type:QtWidgets.QLabel
        self.arm_controls_elbow_faults_label = self.left_screen.arm_controls_elbow_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_roll_comms_label = self.left_screen.arm_controls_roll_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_roll_status_label = self.left_screen.arm_controls_roll_status_label  # type:QtWidgets.QLabel
        self.arm_controls_roll_faults_label = self.left_screen.arm_controls_roll_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_wrist_pitch_comms_label = self.left_screen.arm_controls_wrist_pitch_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_pitch_status_label = self.left_screen.arm_controls_wrist_pitch_status_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_pitch_faults_label = self.left_screen.arm_controls_wrist_pitch_faults_label  # type:QtWidgets.QLabel

        self.arm_controls_wrist_roll_comms_label = self.left_screen.arm_controls_wrist_roll_comms_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_roll_status_label = self.left_screen.arm_controls_wrist_roll_status_label  # type:QtWidgets.QLabel
        self.arm_controls_wrist_roll_faults_label = self.left_screen.arm_controls_wrist_roll_faults_label  # type:QtWidgets.QLabel

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.arm_status_subscriber = rospy.Subscriber(ARM_STATUS_TOPIC, ArmStatusMessage,
                                                      self.new_arm_status_message_received__callback)

        self.arm_relative_control_publisher = rospy.Publisher(ARM_RELATIVE_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)
        self.arm_absolute_control_publisher = rospy.Publisher(ARM_ABSOLUTE_CONTROL_TOPIC, ArmControlMessage,
                                                              queue_size=1)

        self.base_position = 0
        self.shoulder_position = 0
        self.elbow_position = 0
        self.roll_position = 0
        self.wrist_pitch_position = 0
        self.wrist_roll_position = 0

        self.should_stow_arm = False
        self.should_unstow_arm = False

    def run(self):
        self.logger.debug("Starting MiscArm Thread")

        while self.run_thread_flag:
            start_time = time()

            if self.should_stow_arm:
                self.stow_rover_arm()
                self.should_stow_arm = False
            elif self.should_unstow_arm:
                self.unstow_rover_arm()
                self.should_unstow_arm = False

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping MiscArm Thread")

    def stow_rover_arm(self):
        for movement in ARM_STOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def unstow_rover_arm(self):
        for movement in ARM_UNSTOW_PROCEDURE:
            self.process_absolute_move_command(movement)

    def process_absolute_move_command(self, movement):
        message = ArmControlMessage()

        message.base = movement[0]
        message.shoulder = movement[1]
        message.elbow = movement[2]
        message.roll = movement[3]
        message.wrist_pitch = movement[4]
        message.wrist_roll = movement[5]

        print message
        self.arm_absolute_control_publisher.publish(message)

        self.wait_for_targets_reached(movement)

    def wait_for_targets_reached(self, movement):
        base_set = movement[0]
        shoulder_set = movement[1]
        elbow_set = movement[2]
        roll_set = movement[3]
        wrist_pitch_set = movement[4]
        wrist_roll_set = movement[5] - (wrist_pitch_set / 2.0)

        while abs(self.base_position - base_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for base| %f\t%f" % (self.base_position, base_set))
            self.msleep(10)

        while abs(self.shoulder_position - shoulder_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for shoulder| %f\t%f" % (self.shoulder_position, shoulder_set))
            self.msleep(10)

        while abs(self.elbow_position - elbow_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for elbow| %f\t%f" % (self.elbow_position, elbow_set))
            self.msleep(10)

        while abs(self.roll_position - roll_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for roll| %f\t%f" % (self.roll_position, roll_set))
            self.msleep(10)

        while abs(self.wrist_pitch_position - wrist_pitch_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for wrist_pitch| %f\t%f" % (self.wrist_pitch_position, wrist_pitch_set))
            self.msleep(10)

        while abs(self.wrist_roll_position - wrist_roll_set) > POSITIONAL_TOLERANCE:
            self.logger.debug("Waiting for wrist_roll| %f\t%f" % (self.wrist_roll_position, wrist_roll_set))
            self.msleep(10)

    def connect_signals_and_slots(self):
        self.arm_controls_stow_arm_button.clicked.connect(self.on_stow_arm_button_pressed__slot)
        self.arm_controls_unstow_arm_button.clicked.connect(self.on_unstow_arm_button_pressed__slot)
        self.arm_control_upright_zeroed_button.clicked.connect(self.on_upright_zeroed_button_pressed__slot)

        self.arm_controls_calibration_button.clicked.connect(self.on_set_calibration_button_pressed__slot)
        self.arm_controls_clear_faults_button.clicked.connect(self.on_clear_faults_button_pressed__slot)
        self.arm_controls_reset_motor_drivers_button.clicked.connect(self.on_reset_drivers_button_pressed__slot)

        self.base_comms_state_update_ready__signal.connect(self.arm_controls_base_comms_label.setText)
        self.shoulder_comms_state_update_ready__signal.connect(self.arm_controls_shoulder_comms_label.setText)
        self.elbow_comms_state_update_ready__signal.connect(self.arm_controls_elbow_comms_label.setText)
        self.roll_comms_state_update_ready__signal.connect(self.arm_controls_roll_comms_label.setText)
        self.wrist_pitch_comms_state_update_ready__signal.connect(self.arm_controls_wrist_pitch_comms_label.setText)
        self.wrist_roll_comms_state_update_ready__signal.connect(self.arm_controls_wrist_roll_comms_label.setText)

        self.base_status_update_ready__signal.connect(self.arm_controls_base_status_label.setText)
        self.shoulder_status_update_ready__signal.connect(self.arm_controls_shoulder_status_label.setText)
        self.elbow_status_update_ready__signal.connect(self.arm_controls_elbow_status_label.setText)
        self.roll_status_update_ready__signal.connect(self.arm_controls_roll_status_label.setText)
        self.wrist_pitch_status_update_ready__signal.connect(self.arm_controls_wrist_pitch_status_label.setText)
        self.wrist_roll_status_update_ready__signal.connect(self.arm_controls_wrist_roll_status_label.setText)

        self.base_faults_update_ready__signal.connect(self.arm_controls_base_faults_label.setText)
        self.shoulder_faults_update_ready__signal.connect(self.arm_controls_shoulder_faults_label.setText)
        self.elbow_faults_update_ready__signal.connect(self.arm_controls_elbow_faults_label.setText)
        self.roll_faults_update_ready__signal.connect(self.arm_controls_roll_faults_label.setText)
        self.wrist_pitch_faults_update_ready__signal.connect(self.arm_controls_wrist_pitch_faults_label.setText)
        self.wrist_roll_faults_update_ready__signal.connect(self.arm_controls_wrist_roll_faults_label.setText)

    def on_upright_zeroed_button_pressed__slot(self):
        self.process_absolute_move_command([0 for _ in range(6)])

    def on_set_calibration_button_pressed__slot(self):
        message = ArmControlMessage()
        message.calibrate = True
        self.arm_relative_control_publisher.publish(message)

    def on_clear_faults_button_pressed__slot(self):
        message = ArmControlMessage()
        message.clear_faults = True
        self.arm_relative_control_publisher.publish(message)

    def on_reset_drivers_button_pressed__slot(self):
        message = ArmControlMessage()
        message.reset_controllers = True
        self.arm_relative_control_publisher.publish(message)

    def on_stow_arm_button_pressed__slot(self):
        self.should_stow_arm = True

    def on_unstow_arm_button_pressed__slot(self):
        self.should_unstow_arm = True

    def new_arm_status_message_received__callback(self, data):
        self.base_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.base_comm_status))
        self.shoulder_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.shoulder_comm_status))
        self.elbow_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.elbow_comm_status))
        self.roll_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.roll_comm_status))
        self.wrist_pitch_comms_state_update_ready__signal.emit(
            self.process_comms_to_string(data.wrist_pitch_comm_status))
        self.wrist_roll_comms_state_update_ready__signal.emit(self.process_comms_to_string(data.wrist_roll_comm_status))

        self.base_status_update_ready__signal.emit(self.process_statuses_to_string(data.base_status))
        self.shoulder_status_update_ready__signal.emit(self.process_statuses_to_string(data.shoulder_status))
        self.elbow_status_update_ready__signal.emit(self.process_statuses_to_string(data.elbow_status))
        self.roll_status_update_ready__signal.emit(self.process_statuses_to_string(data.roll_status))
        self.wrist_pitch_status_update_ready__signal.emit(self.process_statuses_to_string(data.wrist_pitch_status))
        self.wrist_roll_status_update_ready__signal.emit(self.process_statuses_to_string(data.wrist_roll_status))

        self.base_faults_update_ready__signal.emit(self.process_faults_to_string(data.base_faults))
        self.shoulder_faults_update_ready__signal.emit(self.process_faults_to_string(data.shoulder_faults))
        self.elbow_faults_update_ready__signal.emit(self.process_faults_to_string(data.elbow_faults))
        self.roll_faults_update_ready__signal.emit(self.process_faults_to_string(data.roll_faults))
        self.wrist_pitch_faults_update_ready__signal.emit(self.process_faults_to_string(data.wrist_pitch_faults))
        self.wrist_roll_faults_update_ready__signal.emit(self.process_faults_to_string(data.wrist_roll_faults))

        self.base_position = data.base
        self.shoulder_position = data.shoulder
        self.elbow_position = data.elbow
        self.roll_position = data.roll
        self.wrist_pitch_position = data.wrist_pitch
        self.wrist_roll_position = data.wrist_roll

    @staticmethod
    def process_faults_to_string(faults):
        fault_output = ""

        for bit_position in FAULT_TO_STRING:
            if (1 << bit_position) & faults:
                fault_output += FAULT_TO_STRING[bit_position] + "\n"

        return fault_output[:-1]

    @staticmethod
    def process_statuses_to_string(statuses):
        status_output = ""

        for bit_position in STATUS_TO_STRING:
            if (1 << bit_position) & statuses:
                status_output += STATUS_TO_STRING[bit_position] + "\n"

        return status_output[:-1]

    @staticmethod
    def process_comms_to_string(comms):
        return COMMS_TO_STRING[comms] if comms in COMMS_TO_STRING else "UNKNOWN"

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
