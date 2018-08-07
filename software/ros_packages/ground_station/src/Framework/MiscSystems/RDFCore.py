# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy
from time import time

from std_msgs.msg import Float64MultiArray

#####################################
# Global Variables
#####################################
RDF_DATA_TOPIC = "/rdf/data"

THREAD_HERTZ = 5


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class RDF(QtCore.QThread):

    lcd_number_update_ready__signal = QtCore.pyqtSignal(int)

    def __init__(self, shared_objects):
        super(RDF, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.rssi_lcdnumber = self.left_screen.rssi_lcdnumber  # type:QtWidgets.QLCDNumber

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.wait_time = 1.0 / THREAD_HERTZ

        self.rdf_subscriber = rospy.Subscriber(RDF_DATA_TOPIC, Float64MultiArray, self.new_rdf_message_received__callback)

        self.data = []
        self.data_limit = 100


    def run(self):
        self.logger.debug("Starting RDF Thread")

        while self.run_thread_flag:
            start_time = time()

            temp = list(self.data)
            if temp:
                average = sum(temp) / len(temp)
                self.lcd_number_update_ready__signal.emit(average)

            time_diff = time() - start_time

            self.msleep(max(int(self.wait_time - time_diff), 0))

        self.logger.debug("Stopping RDF Thread")

    def new_rdf_message_received__callback(self, data):
        if len(self.data) >= self.data_limit:
            del self.data[0]

        # if len(self.times) >= self.data_limit:
        #     del self.times[0]

        self.data.append(data.data[0])
        # self.times.append(data.data[1])

    def connect_signals_and_slots(self):
        self.lcd_number_update_ready__signal.connect(self.rssi_lcdnumber.display)

    def setup_signals(self, start_signal, signals_and_slots_signal, kill_signal):
        start_signal.connect(self.start)
        signals_and_slots_signal.connect(self.connect_signals_and_slots)
        kill_signal.connect(self.on_kill_threads_requested__slot)

    def on_kill_threads_requested__slot(self):
        self.run_thread_flag = False
