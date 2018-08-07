#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover_control.msg import MiningControlMessage, MiningStatusMessage, GripperControlMessage, GripperStatusMessage

#####################################
# Global Variables
#####################################
NODE_NAME = "effectors_control"

# ##### Communication Defines #####
# DEFAULT_PORT = "/dev/rover/ttyEffectors"
DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_BAUD = 115200

GRIPPER_NODE_ID = 1
MINING_NODE_ID = 2
SCIENCE_NODE_ID = 3

GRIPPER_TIMEOUT = 0.5
MINING_TIMEOUT = 0.3
SCIENCE_TIMEOUT = 0.15

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 40

GRIPPER_CONTROL_SUBSCRIBER_TOPIC = "gripper/control"
GRIPPER_STATUS_PUBLISHER_TOPIC = "gripper/status"

MINING_CONTROL_SUBSCRIBER_TOPIC = "mining/control"
MINING_STATUS_PUBLISHER_TOPIC = "mining/status"

SCIENCE_CONTROL_SUBSCRIBER_TOPIC = "science/control"

# ##### Gripper Defines #####
GRIPPER_MODBUS_REGISTERS = {
    "MODE": 0,
    "FINGER_POSITION": 1,
    "HOME": 2,
    "LIGHT_STATE": 3,

    "CURRENT_PINCH": 4,
    "CURRENT_FOREFINGER": 5,
    "CURRENT_THUMB": 6,
    "CURRENT_MIDDLEFINGER": 7,
    "POSITION_PINCH": 8,
    "POSITION_FOREFINGER": 9,
    "POSITION_THUMB": 10,
    "POSITION_MIDDLEFINGER": 11,
    "LIGHT_STATE_OUPUT": 12,
    "MODE_OUTPUT": 13,
    "FINGER_POSITION_OUTPUT": 14
}

GRIPPER_MODES = {
    "NO_CHANGE": 0,
    "NORMAL": 1,
    "TWO_FINGER_PINCH": 2,
    "WIDE ": 3,
    "SCISSOR": 4
}

DEFAULT_GRIPPER_REGISTERS = [
    0,  # No change
    0,  # No positional update
    0,  # Do not home
    0,  # Light off
]

# ##### Mining Defines #####
MINING_MODBUS_REGISTERS = {
    "LIFT_SET_POSITIVE": 0,
    "LIFT_SET_NEGATIVE": 1,
    "TILT_SET_POSITIVE": 2,
    "TILT_SET_NEGATIVE": 3,
    "TILT_SET_ABSOLUTE": 4,
    "LIFT_SET_ABSOLUTE": 5,
    "MEASURE": 6,
    "TARE": 7,
    "CAL_FACTOR": 8,

    "LIFT_POSITION": 9,
    "TILT_POSITION": 10,
    "MEASURED_WEIGHT": 11
}

MINING_POSITIONAL_THRESHOLD = 20

# ##### Science Defines #####

# ##### Misc Defines #####
NODE_LAST_SEEN_TIMEOUT = 2  # seconds

UINT16_MAX = 65535


#####################################
# DriveControl Class Definition
#####################################
class EffectorsControl(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.gripper_node_id = rospy.get_param("~gripper_node_id", GRIPPER_NODE_ID)
        self.mining_node_id = rospy.get_param("~mining_node_id", MINING_NODE_ID)
        self.science_node_id = rospy.get_param("~science_node_id", SCIENCE_NODE_ID)

        self.gripper_control_subscriber_topic = rospy.get_param("~gripper_control_subscriber_topic",
                                                                GRIPPER_CONTROL_SUBSCRIBER_TOPIC)
        self.gripper_status_publisher_topic = rospy.get_param("~gripper_status_publisher_topic",
                                                              GRIPPER_STATUS_PUBLISHER_TOPIC)

        self.mining_control_subscriber_topic = rospy.get_param("~mining_control_subscriber_topic",
                                                               MINING_CONTROL_SUBSCRIBER_TOPIC)

        self.mining_status_publisher_topic = rospy.get_param("~mining_status_publisher_topic",
                                                             MINING_STATUS_PUBLISHER_TOPIC)

        self.science_control_subscriber_topic = rospy.get_param("~science_control_subscriber_topic",
                                                                SCIENCE_CONTROL_SUBSCRIBER_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.gripper_node = None  # type:minimalmodbus.Instrument
        self.mining_node = None  # type:minimalmodbus.Instrument
        self.science_node = None  # type:minimalmodbus.Instrument

        self.gripper_node_present = False
        self.mining_node_present = True
        self.science_node_present = False

        self.connect_to_nodes()
        # self.check_which_nodes_present()

        # ##### Subscribers #####
        self.gripper_control_subscriber = rospy.Subscriber(self.gripper_control_subscriber_topic, GripperControlMessage, self.gripper_control_message_received__callback)

        self.mining_control_subscriber = rospy.Subscriber(self.mining_control_subscriber_topic, MiningStatusMessage,
                                                          self.mining_control_message_received__callback)

        # ##### Publishers #####
        self.gripper_status_publisher = rospy.Publisher(self.gripper_status_publisher_topic, GripperStatusMessage, queue_size=1)

        self.mining_status_publisher = rospy.Publisher(self.mining_status_publisher_topic, MiningStatusMessage,
                                                       queue_size=1)

        # ##### Misc #####
        self.modbus_nodes_seen_time = time()

        # ##### Mining Variables #####
        self.mining_registers = [0 for _ in MINING_MODBUS_REGISTERS]
        self.gripper_registers = [0 for _ in GRIPPER_MODBUS_REGISTERS]

        self.mining_control_message = None  # type:MiningControlMessage
        self.new_mining_control_message = False

        self.gripper_control_message = None
        self.new_gripper_control_message = False

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.gripper_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=GRIPPER_TIMEOUT)
        self.gripper_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                         delay_before_rx=RX_DELAY,
                                                                         delay_before_tx=TX_DELAY)

        self.mining_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=MINING_TIMEOUT)
        self.mining_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                        delay_before_rx=RX_DELAY,
                                                                        delay_before_tx=TX_DELAY)

        self.science_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=SCIENCE_TIMEOUT)
        self.science_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                         delay_before_rx=RX_DELAY,
                                                                         delay_before_tx=TX_DELAY)

    def run(self):
        # self.initialize_mining_system()

        try:
            self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))
        except:
            pass

        while not rospy.is_shutdown():
            # try:
            #     self.process_mining_control_message()
            # except IOError, e:
            #     # print e
            #     if (time() - self.modbus_nodes_seen_time) > NODE_LAST_SEEN_TIMEOUT:
            #         print "Lost connection to mining system. Exiting for reconnect."
            #         return
            # except Exception, e:
            #     pass
            #
            # try:
            #     self.send_mining_status_message()
            # except IOError, e:
            #     print e
            #     if (time() - self.modbus_nodes_seen_time) > NODE_LAST_SEEN_TIMEOUT:
            #         print "Lost connection to mining system. Exiting for reconnect."
            #         return
            # except Exception, e:
            #     pass

            try:
                self.process_gripper_control_message()
                self.send_gripper_status_message()
            except Exception, error:
                print error

    def connect_to_nodes(self):
        self.gripper_node = minimalmodbus.Instrument(self.port, int(self.gripper_node_id))
        self.mining_node = minimalmodbus.Instrument(self.port, int(self.mining_node_id))
        self.science_node = minimalmodbus.Instrument(self.port, int(self.science_node_id))

        self.__setup_minimalmodbus_for_485()

    def check_which_nodes_present(self):
        try:
            self.gripper_node.read_register(0)
            self.gripper_node_present = True
        except:
            self.gripper_node_present = False

        try:
            self.mining_node.read_register(0)
            self.mining_node_present = True
        except:
            self.mining_node_present = False

        try:
            self.science_node.read_register(0)
            self.science_node_present = True
        except:
            self.science_node_present = False

    def process_mining_control_message(self):

        if self.new_mining_control_message and self.mining_node_present:
            lift_set_relative = self.mining_control_message.lift_set_relative
            tilt_set_relative = self.mining_control_message.tilt_set_relative
            lift_set_absolute = self.mining_control_message.lift_set_absolute
            tilt_set_absolute = self.mining_control_message.tilt_set_absolute
            cal_factor = min(self.mining_control_message.cal_factor, UINT16_MAX)
            measure = self.mining_control_message.measure
            tare = self.mining_control_message.tare

            if lift_set_absolute < 1024:
                self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_ABSOLUTE"]] = lift_set_absolute
            else:
                if lift_set_relative >= 0:
                    self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_POSITIVE"]] = lift_set_relative
                else:
                    self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_SET_NEGATIVE"]] = -lift_set_relative

            if tilt_set_absolute < 1024:
                self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_ABSOLUTE"]] = tilt_set_absolute
            else:
                if tilt_set_relative >= 0:
                    self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_POSITIVE"]] = tilt_set_relative
                else:
                    self.mining_registers[MINING_MODBUS_REGISTERS["TILT_SET_NEGATIVE"]] = -tilt_set_relative

            if cal_factor > -1:
                self.mining_registers[MINING_MODBUS_REGISTERS["CAL_FACTOR"]] = cal_factor

            self.mining_registers[MINING_MODBUS_REGISTERS["MEASURE"]] = int(measure)
            self.mining_registers[MINING_MODBUS_REGISTERS["TARE"]] = int(tare)

            self.mining_node.write_registers(0, self.mining_registers)

            self.modbus_nodes_seen_time = time()
            self.new_mining_control_message = False

    def send_mining_status_message(self):
        if self.mining_node_present:
            self.mining_registers = self.mining_node.read_registers(0, len(MINING_MODBUS_REGISTERS))

            message = MiningStatusMessage()
            message.lift_position = self.mining_registers[MINING_MODBUS_REGISTERS["LIFT_POSITION"]]
            message.tilt_position = self.mining_registers[MINING_MODBUS_REGISTERS["TILT_POSITION"]]
            message.measured_weight = self.mining_registers[MINING_MODBUS_REGISTERS["MEASURED_WEIGHT"]]

            self.mining_status_publisher.publish(message)

            self.modbus_nodes_seen_time = time()

    def process_gripper_control_message(self):
        if self.new_gripper_control_message:

            if self.gripper_control_message.toggle_light:
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE"]] = not self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE"]]

            self.gripper_registers[GRIPPER_MODBUS_REGISTERS["HOME"]] = self.gripper_control_message.should_home
            self.gripper_registers[GRIPPER_MODBUS_REGISTERS["MODE"]] = self.gripper_control_message.gripper_mode

            gripper_absolute = self.gripper_control_message.gripper_position_absolute
            gripper_relative = self.gripper_control_message.gripper_position_relative

            if -1 < gripper_absolute < UINT16_MAX:
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] = gripper_absolute
            else:
                new_position = self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] + gripper_relative
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION"]] = min(max(new_position, 0), 10000)

            self.gripper_node.write_registers(0, self.gripper_registers)

            self.gripper_node.write_registers(0, DEFAULT_GRIPPER_REGISTERS)

            self.new_gripper_control_message = False

    def send_gripper_status_message(self):
        registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))

        message = GripperStatusMessage()
        message.pinch_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_PINCH"]]
        message.forefinger_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_FOREFINGER"]]
        message.thumb_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_THUMB"]]
        message.middlefinger_position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION_MIDDLEFINGER"]]
        message.pinch_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_PINCH"]]
        message.forefinger_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_FOREFINGER"]]
        message.thumb_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_THUMB"]]
        message.middlefinger_current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT_MIDDLEFINGER"]]
        message.light_on = registers[GRIPPER_MODBUS_REGISTERS["LIGHT_STATE_OUPUT"]]
        message.current_mode = registers[GRIPPER_MODBUS_REGISTERS["MODE_OUTPUT"]]
        message.current_finger_position = registers[GRIPPER_MODBUS_REGISTERS["FINGER_POSITION_OUTPUT"]]

        self.gripper_status_publisher.publish(message)

    def gripper_control_message_received__callback(self, control_message):
        self.gripper_control_message = control_message
        self.new_gripper_control_message = True

    def mining_control_message_received__callback(self, control_message):
        self.mining_control_message = control_message
        self.new_mining_control_message = True


if __name__ == "__main__":
    EffectorsControl()
