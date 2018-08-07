#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy

from time import time, sleep

import serial.rs485
import minimalmodbus
import numpy

# Custom Imports
from rover_control.msg import TowerPanTiltControlMessage
from std_msgs.msg import Float64MultiArray

#####################################
# Global Variables
#####################################
NODE_NAME = "science_node"

DEFAULT_PORT = "/dev/rover/ttyRDF_SoilProbe"
DEFAULT_BAUD = 9600

DEFAULT_INVERT = False

DEFAULT_RDF_PUBLISHER_TOPIC = "rdf/data"

PAN_TILT_NODE_ID = 1

COMMUNICATIONS_TIMEOUT = 0.1  # Seconds

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 20

SOIL_PROBE_ADDRESS = "mar"

PAN_TILT_MODBUS_REGISTERS = {
    "CENTER_ALL": 0,

    "PAN_ADJUST_POSITIVE": 1,
    "PAN_ADJUST_NEGATIVE": 2,
    "TILT_ADJUST_POSITIVE": 3,
    "TILT_ADJUST_NEGATIVE": 4
}

SOIL_PROBE_COMMANDS = {
    "GET_ADDRESS": "AD=",
    "DESCRIPTION": "DS=",
    "PROBE_ENABLED": "PE=",
    "TAKE_READING": "TR",
    "TRANSMIT_READING": "T0",

    "QUERY": "?"
}

NODE_LAST_SEEN_TIMEOUT = 2  # seconds


#####################################
# DriveControl Class Definition
#####################################
class RoverScience(object):
    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.pan_tilt_node_id = rospy.get_param("~pan_tilt_node_id", PAN_TILT_NODE_ID)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.pan_tilt_node = None
        self.tower_node = None

        self.connect_to_pan_tilt_and_tower()

        self.rdf_publisher = rospy.Publisher(DEFAULT_RDF_PUBLISHER_TOPIC, Float64MultiArray, queue_size=1)

        self.pan_tilt_control_message = None
        self.new_pan_tilt_control_message = False

        self.modbus_nodes_seen_time = time()

        self.data = numpy.array([])
        self.times = numpy.array([])
        self.max_data_len = 200

        self.count = 0
        self.start_time = time()

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.pan_tilt_node.serial = serial.rs485.RS485(self.port, baudrate=self.baud, timeout=COMMUNICATIONS_TIMEOUT)  # type: serial.Serial
        self.pan_tilt_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0,
                                                                          delay_before_rx=RX_DELAY,
                                                                          delay_before_tx=TX_DELAY)

    def run(self):
        # self.send_startup_centering_command()
        # self.pan_tilt_node.serial.timeout(500)

        out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["PROBE_ENABLED"] + "1")
        self.pan_tilt_node.serial.write(out)
        sleep(0.1)

        while not rospy.is_shutdown():

            out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["TAKE_READING"])
            self.pan_tilt_node.serial.write(out)
            sleep(0.1)

            out = "%s%s\r\n" % (SOIL_PROBE_ADDRESS, SOIL_PROBE_COMMANDS["TRANSMIT_READING"])
            self.pan_tilt_node.serial.write(out)

            start_time = time()
            char = ""
            response = ""
            while char != '\n' and (time() - start_time) < 2:
                # print time() - start_time, (time() - start_time) > 2
                if self.pan_tilt_node.serial.inWaiting():
                    char = self.pan_tilt_node.serial.read()
                    response += char
                    # print char

            if response:
                print response
            else:
                print "timeout"
                # print "No response"

            # start_time = time()
            # try:
            #     registers = self.pan_tilt_node.read_registers(0, 1)
            #     self.rdf_publisher.publish(Float64MultiArray(data=[registers[0], time()]))
            #     self.modbus_nodes_seen_time = time()
            # except Exception, Error:
            #     print Error
            #
            # if (time() - self.modbus_nodes_seen_time) > NODE_LAST_SEEN_TIMEOUT:
            #     print "Science not seen for", NODE_LAST_SEEN_TIMEOUT, "seconds. Exiting."
            #     return  # Exit so respawn can take over

            # time_diff = time() - start_time

    # def run(self):
    #     # self.send_startup_centering_command()
    #     while not rospy.is_shutdown():
    #         registers = self.pan_tilt_node.read_registers(0, 1)
    #         self.data = numpy.append(self.data, registers[0])
    #         self.times = numpy.append(self.times, time())
    #
    #         if len(self.data) > self.max_data_len:
    #             numpy.delete(self.data, 0)
    #             numpy.delete(self.times, 0)
    #
    #             # for item in self.smoothListTriangle(self.data):
    #             #     print item
    #
    #             print fq.freq_from_fft(self.data, 1/40.0)
    #             # self.data = self.smoothListGaussian(self.data)
    #             #
    #             # w = numpy.fft.fft(self.data)
    #             #
    #             # # for item in w:
    #             # #     print abs(item)
    #             # #
    #             # # print
    #             # # print
    #             # numpy.delete(w, 0)
    #             # freqs = numpy.fft.fftfreq(len(w), 1/40.0)
    #             # # # print len(freqs), len(w)
    #             # #
    #             # # # print freqs
    #             # # # print(freqs.min(), freqs.max())
    #             # # # # (-0.5, 0.499975)
    #             # # #
    #             # # # # Find the peak in the coefficients
    #             # idx = numpy.argmax(numpy.abs(w))
    #             # freq = freqs[idx]
    #             # freq_in_hertz = abs(freq * 40)
    #             # print(freq_in_hertz)


    def smoothListTriangle(self, list, strippedXs=False, degree=5):

        weight = []

        window = degree * 2 - 1

        smoothed = [0.0] * (len(list) - window)

        for x in range(1, 2 * degree): weight.append(degree - abs(degree - x))

        w = numpy.array(weight)

        for i in range(len(smoothed)):
            smoothed[i] = sum(numpy.array(list[i:i + window]) * w) / float(sum(w))

        return smoothed

    def smoothListGaussian(self, list, strippedXs=False, degree=5):

        window = degree * 2 - 1

        weight = numpy.array([1.0] * window)

        weightGauss = []

        for i in range(window):
            i = i - degree + 1

            frac = i / float(window)

            gauss = 1 / (numpy.exp((4 * (frac)) ** 2))

            weightGauss.append(gauss)

        weight = numpy.array(weightGauss) * weight

        smoothed = [0.0] * (len(list) - window)

        for i in range(len(smoothed)):
            smoothed[i] = sum(numpy.array(list[i:i + window]) * weight) / sum(weight)

        return smoothed

    def connect_to_pan_tilt_and_tower(self):
        self.pan_tilt_node = minimalmodbus.Instrument(self.port, int(self.pan_tilt_node_id))
        self.__setup_minimalmodbus_for_485()


if __name__ == "__main__":
    RoverScience()
