#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import serial.rs485
import minimalmodbus
import time

import rospy
from rover_drive.msg import RoverMotorDrive

# Custom Imports

#####################################
# Global Variables
#####################################
NODE_NAME = "rover_drive"

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 2000000

MOTOR_TOPIC = "motoroneandtwo"

FIRST_MOTOR_ID = 1
SECOND_MOTOR_ID = 2

COMMUNICATIONS_TIMEOUT = 0.3  # Seconds

MOTOR_DRIVERS_DIRECTION = {
    "FORWARDS": 1,
    "BACKWARDS": 0
}

MOTOR_DRIVER_REGISTER_MAP = {
    "DIRECTION": 0,
    "SPEED": 1
}

MOTOR_DRIVER_DEFAULT_MESSAGE = [
    MOTOR_DRIVERS_DIRECTION["FORWARDS"],
    0
]


#####################################
# RoverDrive Class Definition
#####################################
class RoverDrive(object):
    def __init__(self):

        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("port", DEFAULT_PORT)
        self.baud = rospy.get_param("baud", DEFAULT_BAUD)

        self.first_motor_modbus_id = rospy.get_param("first_motor_modbus_id", FIRST_MOTOR_ID)
        self.second_motor_modbus_id = rospy.get_param("second_motor_modbus_id", SECOND_MOTOR_ID)

        self.motors_subscriber_topic = rospy.get_param("topic", MOTOR_TOPIC)

        self.first_motor = minimalmodbus.Instrument(self.port, self.first_motor_modbus_id)
        self.second_motor = minimalmodbus.Instrument(self.port, self.second_motor_modbus_id)

        self.motors_subscriber = rospy.Subscriber(MOTOR_TOPIC, RoverMotorDrive, self.__motor_message_callback)

        self.__setup_minimalmodbus_for_485()

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            time.sleep(0.25)
            # try:
            #     self.first_motor.write_register(1, 10000)
            # except IOError, error:
            #     pass
            #
            # try:
            #     self.second_motor.write_register(1, 30000)
            # except IOError, error:
            #     pass

    def __setup_minimalmodbus_for_485(self):
        self.first_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.first_motor.serial.rs485_mode = \
            serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=0, delay_before_tx=0)

        self.second_motor.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
        self.second_motor.serial.rs485_mode = \
            serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=0, delay_before_tx=0)

    def __motor_message_callback(self, data):
        first_motor_register_data = list(MOTOR_DRIVER_DEFAULT_MESSAGE)  # Makes a copy
        first_motor_register_data[MOTOR_DRIVER_REGISTER_MAP["DIRECTION"]] = int(data.first_motor_direction)
        first_motor_register_data[MOTOR_DRIVER_REGISTER_MAP["SPEED"]] = data.first_motor_speed
        print first_motor_register_data
        self.first_motor.write_registers(0, first_motor_register_data)

#####################################
# Main Definition
#####################################
if __name__ == "__main__":
    drive = RoverDrive()
