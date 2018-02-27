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
from rover_control.msg import DriveControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "drive_control"

DEFAULT_PORT = "/dev/rover/ttyBogie"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_DRIVE_CONTROL_TOPIC = "drive_control/rear"

FIRST_MOTOR_ID = 1
SECOND_MOTOR_ID = 2

COMMUNICATIONS_TIMEOUT = 0.15  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1,
    "SLEEP": 2,

    "CURRENT": 3,
    "FAULT": 4,

    "TEMPERATURE": 5
}

MOTOR_DRIVER_DEFAULT_MESSAGE = [
    1,  # Forwards
    0,  # 0 Speed
    1  # Not in sleep mode
]

UINT16_MAX = 65535

#####################################
# DriveControl Class Definition
#####################################
class DriveControl(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.first_motor_inverted = rospy.get_param("~invert_first_motor", DEFAULT_INVERT)
        self.second_motor_inverted = rospy.get_param("~invert_second_motor", DEFAULT_INVERT)

        self.drive_control_subscriber_topic = rospy.get_param("~drive_control_topic", DEFAULT_DRIVE_CONTROL_TOPIC)

        self.first_motor = minimalmodbus.Instrument(self.port, FIRST_MOTOR_ID)
        self.second_motor = minimalmodbus.Instrument(self.port, SECOND_MOTOR_ID)
        self.__setup_minimalmodbus_for_485()

        self.drive_control_subscriber = \
            rospy.Subscriber(self.drive_control_subscriber_topic, DriveControlMessage, self.drive_control_callback)

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.first_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.first_motor.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                        delay_before_rx=RX_DELAY,
                                                                        delay_before_tx=TX_DELAY)

        self.second_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.second_motor.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                         delay_before_rx=RX_DELAY,
                                                                         delay_before_tx=TX_DELAY)

    def run(self):
        while not rospy.is_shutdown():
            sleep(0.25)

    def drive_control_callback(self, drive_control):
        try:
            first_motor_register_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
            first_direction = \
                not drive_control.first_motor_direction if self.first_motor_inverted else drive_control.first_motor_direction
            first_motor_register_data[MODBUS_REGISTERS["DIRECTION"]] = first_direction
            first_motor_register_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.first_motor_speed, UINT16_MAX)

            second_motor_register_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)
            second_direction = not drive_control.second_motor_direction if self.second_motor_inverted else drive_control.second_motor_direction
            second_motor_register_data[MODBUS_REGISTERS["DIRECTION"]] = second_direction
            second_motor_register_data[MODBUS_REGISTERS["SPEED"]] = min(drive_control.second_motor_speed, UINT16_MAX)

            self.first_motor.write_registers(0, first_motor_register_data)
            self.second_motor.write_registers(0, second_motor_register_data)

        except Exception, error:
            print "Error occurred:", error


if __name__ == "__main__":
    DriveControl()
