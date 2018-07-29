#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports

from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
# from rover_control.msg import TowerPanTiltControlMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "chassis_pan_tilt_control"

DEFAULT_PORT = "/dev/rover/ttyChassisPanTilt"
DEFAULT_BAUD = 115200

DEFAULT_INVERT = False

DEFAULT_PAN_TILT_CONTROL_TOPIC = "chassis/pan_tilt/control"

PAN_TILT_NODE_ID = 1

COMMUNICATIONS_TIMEOUT = 0.01  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 20

PAN_TILT_MODBUS_REGISTERS = {
    "CENTER_ALL": 0,

    "PAN_ADJUST_POSITIVE": 1,
    "PAN_ADJUST_NEGATIVE": 2,
    "TILT_ADJUST_POSITIVE": 3,
    "TILT_ADJUST_NEGATIVE": 4
}

PAN_TILT_CONTROL_DEFAULT_MESSAGE = [
    0,  # No centering
    0,  # No pan positive adjustment
    0,  # No pan negative adjustment
    0,  # No tilt positive adjustment
    0  # No tilt negative adjustement
]

self.mining_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)
self.mining_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                delay_before_rx=RX_DELAY,
                                                                delay_before_tx=TX_DELAY)


def run(self):
    print(self.mining_node.read_registers(0, 7))


def connect_to_pan_tilt_and_tower(self):
    self.mining_node = minimalmodbus.Instrument(self.port, int(2))
    self.__setup_minimalmodbus_for_485()


NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class MiningControl(object):
    def __init__(self):
        self.port = "COM22"
        self.baud = 115200

        self.mining_node = None
        self.tower_node = None

        self.connect_to_pan_tilt_and_tower()

        self.pan_tilt_control_message = None
        self.new_pan_tilt_control_message = False

        self.modbus_nodes_seen_time = time()

        self.run()

    def __setup_minimalmodbus_for_485(self):
if __name__ == "__main__":
    MiningControl()
