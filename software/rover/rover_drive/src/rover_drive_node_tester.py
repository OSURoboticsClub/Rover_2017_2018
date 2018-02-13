import rospy
import time
import atexit
import signal

from rover_drive.msg import RoverMotorDrive

signal.signal(signal.SIGINT, signal.SIG_DFL)

rospy.init_node("drive_tester")

pub = rospy.Publisher("/drive/motoroneandtwo", RoverMotorDrive, queue_size=1)

speed_step = 1000

last_dir = 0

sleep_time = 0.1


def stop_motors():
    global pub

    drive.first_motor_direction = last_dir
    drive.first_motor_speed = 0
    pub.publish(drive)

atexit.register(stop_motors)

while not rospy.is_shutdown():
    try :
        drive = RoverMotorDrive()

        last_dir = not last_dir
        for i in range(0, 65535, speed_step):
            drive.first_motor_direction = last_dir
            drive.first_motor_speed = i
            pub.publish(drive)
            time.sleep(sleep_time)
            print i
        for i in range(65535, 0, -speed_step):
            drive.first_motor_direction = last_dir
            drive.first_motor_speed = i
            pub.publish(drive)
            time.sleep(sleep_time)
            print i
    except KeyboardInterrupt:
        exit()


