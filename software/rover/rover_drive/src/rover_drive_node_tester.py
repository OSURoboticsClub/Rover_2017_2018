import rospy
import time

from rover_drive.msg import RoverMotorDrive

rospy.init_node("drive_tester")

pub = rospy.Publisher("/drive/motoroneandtwo", RoverMotorDrive, queue_size=10)

speed_step = 500

last_dir = 0

sleep_time = 0.01

while not rospy.is_shutdown():
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
