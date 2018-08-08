# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets
import logging
import rospy

from rover_control.msg import MiningStatusMessage, MiningControlMessage
from rover_science.msg import SoilSensorStatusMessage

#####################################
# Global Variables
#####################################
MINING_STATUS_TOPIC = "/rover_control/mining/status"
MINING_CONTROL_TOPIC = "/rover_control/mining/control"

SOIL_PROBE_TOPIC = "/rover_science/soil_probe/data"

TRAVEL_POSITION_LIFT = 110
TRAVEL_POSITION_TILT = 1023

MEASURE_POSITION_LIFT = 350
MEASURE_POSITION_TILT = 1023

SCOOP_POSITION_LIFT = 228
SCOOP_POSITION_TILT = 215


#####################################
# UbiquitiRadioSettings Class Definition
#####################################
class Mining(QtCore.QObject):

    lift_position_update_ready__signal = QtCore.pyqtSignal(int)
    tilt_position_update_ready__signal = QtCore.pyqtSignal(int)

    weight_measurement_update_ready__signal = QtCore.pyqtSignal(int)

    temp_update_ready__signal = QtCore.pyqtSignal(float)
    moisture_update_ready__signal = QtCore.pyqtSignal(float)
    loss_tangent_update_ready__signal = QtCore.pyqtSignal(float)
    electrical_conductivity_update_ready__signal = QtCore.pyqtSignal(float)
    real_dielectric_update_ready__signal = QtCore.pyqtSignal(float)
    imaginary_dielectric_update_ready__signal = QtCore.pyqtSignal(float)

    def __init__(self, shared_objects):
        super(Mining, self).__init__()

        # ########## Reference to class init variables ##########
        self.shared_objects = shared_objects
        self.left_screen = self.shared_objects["screens"]["left_screen"]

        self.mining_qlcdnumber = self.left_screen.mining_qlcdnumber  # type:QtWidgets.QLCDNumber
        self.mining_tare_button = self.left_screen.mining_tare_button  # type:QtWidgets.QPushButton
        self.mining_measure_button = self.left_screen.mining_measure_button  # type:QtWidgets.QPushButton
        self.mining_cal_factor_spinbox = self.left_screen.mining_cal_factor_spinbox  # type:QtWidgets.QSpinBox
        self.mining_set_cal_factor_button = self.left_screen.mining_set_cal_factor_button  # type:QtWidgets.QPushButton
        self.lift_position_progress_bar = self.left_screen.lift_position_progress_bar  # type:QtWidgets.QProgressBar
        self.tilt_position_progress_bar = self.left_screen.tilt_position_progress_bar  # type:QtWidgets.QProgressBar

        self.mining_measure_move_button = self.left_screen.mining_measure_move_button  # type:QtWidgets.QPushButton
        self.mining_transport_move_button = self.left_screen.mining_transport_move_button  # type:QtWidgets.QPushButton
        self.mining_scoop_move_button = self.left_screen.mining_scoop_move_button  # type:QtWidgets.QPushButton

        self.science_temp_lcd_number = self.left_screen.science_temp_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_moisture_lcd_number = self.left_screen.science_moisture_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_loss_tangent_lcd_number = self.left_screen.science_loss_tangent_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_electrical_conductivity_lcd_number = self.left_screen.science_electrical_conductivity_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_real_dielectric_lcd_number = self.left_screen.science_real_dielectric_lcd_number  # type:QtWidgets.QLCDNumber
        self.science_imaginary_dielectric_lcd_number = self.left_screen.science_imaginary_dielectric_lcd_number  # type:QtWidgets.QLCDNumber

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Get the Pick And Plate instance of the logger ##########
        self.logger = logging.getLogger("groundstation")

        # ########## Thread Flags ##########
        self.run_thread_flag = True

        # ########## Class Variables ##########
        self.mining_status_subscriber = rospy.Subscriber(MINING_STATUS_TOPIC, MiningStatusMessage,
                                                         self.mining_status_message_received__callback)

        self.soil_probe_subscriber = rospy.Subscriber(SOIL_PROBE_TOPIC, SoilSensorStatusMessage, self.on_soil_probe_message_received__callback)

        self.mining_control_publisher = rospy.Publisher(MINING_CONTROL_TOPIC, MiningControlMessage, queue_size=1)

        self.connect_signals_and_slots()

    def connect_signals_and_slots(self):
        self.mining_set_cal_factor_button.clicked.connect(self.on_mining_set_cal_factor_clicked__slot)
        self.mining_tare_button.clicked.connect(self.on_mining_tare_clicked__slot)
        self.mining_measure_button.clicked.connect(self.on_mining_measure_clicked__slot)

        self.mining_measure_move_button.clicked.connect(self.on_mining_move_measure_clicked__slot)
        self.mining_transport_move_button.clicked.connect(self.on_mining_move_transport_clicked__slot)
        self.mining_scoop_move_button.clicked.connect(self.on_mining_move_scoop_clicked__slot)

        self.tilt_position_update_ready__signal.connect(self.tilt_position_progress_bar.setValue)
        self.lift_position_update_ready__signal.connect(self.lift_position_progress_bar.setValue)

        self.weight_measurement_update_ready__signal.connect(self.mining_qlcdnumber.display)

        self.temp_update_ready__signal.connect(self.science_temp_lcd_number.display)
        self.moisture_update_ready__signal.connect(self.science_moisture_lcd_number.display)
        self.loss_tangent_update_ready__signal.connect(self.science_loss_tangent_lcd_number.display)
        self.electrical_conductivity_update_ready__signal.connect(self.science_electrical_conductivity_lcd_number.display)
        self.real_dielectric_update_ready__signal.connect(self.science_real_dielectric_lcd_number.display)
        self.imaginary_dielectric_update_ready__signal.connect(self.science_imaginary_dielectric_lcd_number.display)

    def on_mining_set_cal_factor_clicked__slot(self):
        message = MiningControlMessage()

        message.tilt_set_absolute = 1024
        message.lift_set_absolute = 1024
        message.cal_factor = self.mining_cal_factor_spinbox.value()

        self.mining_control_publisher.publish(message)

    def on_mining_tare_clicked__slot(self):
        message = MiningControlMessage()
        message.tilt_set_absolute = 1024
        message.lift_set_absolute = 1024
        message.cal_factor = -1
        message.tare = 1

        self.mining_control_publisher.publish(message)

    def on_mining_measure_clicked__slot(self):
        message = MiningControlMessage()
        message.tilt_set_absolute = 1024
        message.lift_set_absolute = 1024
        message.cal_factor = -1
        message.measure = True

        self.mining_control_publisher.publish(message)

    def on_mining_move_transport_clicked__slot(self):
        message = MiningControlMessage()
        message.tilt_set_absolute = TRAVEL_POSITION_TILT
        message.lift_set_absolute = TRAVEL_POSITION_LIFT
        message.cal_factor = -1

        self.mining_control_publisher.publish(message)

    def on_mining_move_measure_clicked__slot(self):
        message = MiningControlMessage()
        message.tilt_set_absolute = MEASURE_POSITION_TILT
        message.lift_set_absolute = MEASURE_POSITION_LIFT
        message.cal_factor = -1

        self.mining_control_publisher.publish(message)

    def on_mining_move_scoop_clicked__slot(self):
        message = MiningControlMessage()
        message.tilt_set_absolute = SCOOP_POSITION_TILT
        message.lift_set_absolute = SCOOP_POSITION_LIFT
        message.cal_factor = -1

        self.mining_control_publisher.publish(message)

    def mining_status_message_received__callback(self, status):
        status = status  # type:MiningStatusMessage
        self.tilt_position_update_ready__signal.emit(status.tilt_position)
        self.lift_position_update_ready__signal.emit(status.lift_position)
        self.weight_measurement_update_ready__signal.emit(status.measured_weight)

    def on_soil_probe_message_received__callback(self, data):
        self.temp_update_ready__signal.emit(data.temp_c)
        self.moisture_update_ready__signal.emit(data.moisture)
        self.loss_tangent_update_ready__signal.emit(data.loss_tangent)
        self.electrical_conductivity_update_ready__signal.emit(data.soil_electrical_conductivity)
        self.real_dielectric_update_ready__signal.emit(data.real_dielectric_permittivity)
        self.imaginary_dielectric_update_ready__signal.emit(data.imaginary_dielectric_permittivity)
