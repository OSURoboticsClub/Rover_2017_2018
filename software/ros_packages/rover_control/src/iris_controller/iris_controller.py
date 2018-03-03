#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep

import serial.rs485
import minimalmodbus

# Custom Imports
from rover_control.msg import DriveCommandMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "iris_controller"

DEFAULT_PORT = "/dev/rover/ttyIRIS"
DEFAULT_BAUD = 115200

DEFAULT_DRIVE_COMMAND_TOPIC = "command_control/iris_drive"

DEFAULT_HERTZ = 10
COMMUNICATIONS_TIMEOUT = 0.15  # Seconds

MODBUS_ID = 1

RX_DELAY = 0.01
TX_DELAY = 0.01

SBUS_VALUES = {
    "SBUS_MAX": 1811,
    "SBUS_MID": 991,
    "SBUS_MIN": 172,
    "SBUS_RANGE": 820.0,

    "SBUS_DEADZONE": 5
}

MODBUS_REGISTERS = {
    "LEFT_STICK_Y_AXIS": 0,
    "RIGHT_STICK_Y_AXIS": 1,
    "RIGHT_STICK_X_AXIS": 2,
    "LEFT_STICK_X_AXIS": 3,
    "LEFT_POT": 4,
    "S1_POT": 5,
    "S2_POT": 6,
    "RIGHT_POT": 7,
    "SA_SWITCH": 8,
    "SB_SWITCH": 9,
    "SC_SWITCH": 10,
    "SD_SWITCH": 11,
    "SE_SWITCH": 12,
    "SF_SWITCH": 13,
    "SG_SWITCH": 14,
    "SH_SWITCH": 15,

    "VOLTAGE_24V": 16,
    "VOLTAGE_5V": 17,
    "USB_VOLTAGE_5V": 18,
    "VOLTAGE_3V3": 19
}

REGISTER_STATE_MAPPING = {
    "IGNORE_CONTROL": "SF_SWITCH",
    "DRIVE_VS_ARM": "SE_SWITCH"
}


#####################################
# IrisController Class Definition
#####################################
class IrisController(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.drive_command_publisher_topic = rospy.get_param("~drive_command_topic", DEFAULT_DRIVE_COMMAND_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)
        # print self.wait_time

        self.iris = minimalmodbus.Instrument(self.port, MODBUS_ID)
        self.__setup_minimalmodbus_for_485()

        self.drive_command_publisher = rospy.Publisher(self.drive_command_publisher_topic, DriveCommandMessage,
                                                       queue_size=1)

        self.registers = []

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.iris.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.iris.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                 delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

    def run(self):
        while not rospy.is_shutdown():
            start_time = time()

            try:
                self.read_registers()
                self.broadcast_drive_if_current_mode()
                self.broadcast_arm_if_current_mode()

            except Exception, error:
                print "Error occurred:", error

            time_diff = time() - start_time

            sleep(max(self.wait_time - time_diff, 0))

    def read_registers(self):
        self.registers = self.iris.read_registers(0, len(MODBUS_REGISTERS))
        # print self.registers

    def broadcast_drive_if_current_mode(self):
        if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["DRIVE_VS_ARM"]]] < SBUS_VALUES["SBUS_MID"]:
            command = DriveCommandMessage()

            left_y_axis = self.registers[MODBUS_REGISTERS["LEFT_STICK_Y_AXIS"]]
            right_y_axis = self.registers[MODBUS_REGISTERS["RIGHT_STICK_Y_AXIS"]]

            if left_y_axis == 0 and right_y_axis == 0:
                command.ignore_drive_control = True
                command.drive_twist.linear.x = 0
                command.drive_twist.angular.z = 0
            else:

                left = (left_y_axis - SBUS_VALUES["SBUS_MID"]) / SBUS_VALUES[
                    "SBUS_RANGE"]

                right = (right_y_axis - SBUS_VALUES["SBUS_MID"]) / SBUS_VALUES[
                    "SBUS_RANGE"]

                command.ignore_drive_control = \
                    self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["IGNORE_CONTROL"]]] > SBUS_VALUES["SBUS_MID"]
                command.drive_twist.linear.x = (left + right) / 2.0
                command.drive_twist.angular.z = (right - left) / 2.0

            self.drive_command_publisher.publish(command)

    def broadcast_arm_if_current_mode(self):
        if self.registers[MODBUS_REGISTERS[REGISTER_STATE_MAPPING["DRIVE_VS_ARM"]]] > \
                        SBUS_VALUES["SBUS_MIN"] + SBUS_VALUES["SBUS_DEADZONE"]:
            print "Arm"


if __name__ == "__main__":
    IrisController()