import rospy
import time
import atexit
import signal

import serial.rs485
import minimalmodbus

from rover_drive.msg import RoverMotorDrive

signal.signal(signal.SIGINT, signal.SIG_DFL)

rospy.init_node("drive_tester")

pub = rospy.Publisher("/drive/motoroneandtwo", RoverMotorDrive, queue_size=1)


def make_instr(port_name, baud):
    instr = minimalmodbus.Instrument(port_name, 1)
    instr.serial = serial.rs485.RS485(port_name, baudrate=baud, timeout=0.05)
    instr.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=0,
                                                   delay_before_tx=0)
    return instr

sbus = make_instr("/dev/rover/ttyIRIS_2_2", 2000000)

max = 1811
mid = 990
min = 172

scalar = 75

drive = RoverMotorDrive()
while not rospy.is_shutdown():
    left, right = sbus.read_registers(0, 2)
    # print left
    drive.first_motor_direction = 1 if left > mid else 0
    drive.first_motor_speed = abs(left - mid ) * scalar
    # print drive.first_motor_speed
    pub.publish(drive)
    # try:
    #     drive = RoverMotorDrive()
    #
    #     last_dir = not last_dir
    #     for i in range(0, 65535, speed_step):
    #         drive.first_motor_direction = last_dir
    #         drive.first_motor_speed = i
    #         pub.publish(drive)
    #         time.sleep(sleep_time)
    #         print i
    #     for i in range(65535, 0, -speed_step):
    #         drive.first_motor_direction = last_dir
    #         drive.first_motor_speed = i
    #         pub.publish(drive)
    #         time.sleep(sleep_time)
    #         print i
    # except KeyboardInterrupt:
    #     exit()


