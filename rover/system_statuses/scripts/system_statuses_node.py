#!/usr/bin/env python
import rospy, os.path
from system_statuses.msg import RoverSysStatus, Camera2Changer


class SystemStatuses:

    def __init__(self):
        self.system_path_locations = [
            '/dev/rover/camera_zed',
            '/dev/rover/camera_undercarriage',
            '/dev/rover/camera_gimbal',
            '/dev/rover/camera_main_navigation'
        ]

        rospy.init_node('SystemStatuses')

        self.pub = rospy.Publisher('rover_system_status_chatter', RoverSysStatus, queue_size=10)

        # Test Subscription for updating camera_undercarriage
        rospy.Subscriber('camera_2_changer_chatter', Camera2Changer, self.__sub_callback)

        self.msg = RoverSysStatus()
        # Check if camera_zed is found
        self.msg.camera_zed = 1 if os.path.exists(self.system_path_locations[0]) else 0
        # Check if camera_undercarriage is found
        self.msg.camera_undercarriage = 1 if os.path.exists(self.system_path_locations[1]) else 0
        # Check if camera_gimbal is found
        self.msg.camera_gimbal = 1 if os.path.exists(self.system_path_locations[2]) else 0
        # Check if camera_main_navigation is found
        self.msg.camera_main_navigation = 1 if os.path.exists(self.system_path_locations[3]) else 0

        # self.msg.camera_zed = 0
        # self.msg.camera_undercarriage = 0
        # self.msg.camera_gimbal = 0
        # self.msg.camera_main_navigation = 0

        # Simple Subscriber test boolean -- camera_2_updated.py
        self.hasCameraUndercarriageChanged = False

    # Callback for test Subscription of camera_2_updater.py -- checks if camera_undercarriage
    # has changed, updates value, and updates bool for run->pub.publish to execute
    def __sub_callback(self, data):
        rospy.loginfo("Camera_undercarriage Status Changed: %d " % data.camera_2_value)
        if self.msg.camera_undercarriage != data.camera_2_value:
            self.msg.camera_undercarriage = data.camera_2_value
            self.hasCameraUndercarriageChanged = True

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.hasCameraUndercarriageChanged:
                rospy.loginfo(self.msg)
                self.pub.publish(self.msg)
                self.hasCameraUndercarriageChanged = False
            r.sleep()


if __name__ == '__main__':
    system_status = SystemStatuses()
    system_status.run()
