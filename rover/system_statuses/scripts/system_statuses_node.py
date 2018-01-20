#!/usr/bin/env python
import rospy, os.path
from system_statuses.msg import RoverSysStatus, Camera2Changer


class SystemStatuses:

    def __init__(self):
        self.system_path_locations = [
            '/dev/rover/camera_zed',
            '/dev/rover/camera_undercarriage',
            '/dev/rover/camera_chassis',
            '/dev/rover/camera_main_navigation'
        ]

        rospy.init_node('SystemStatuses')

        self.pub = rospy.Publisher('rover_system_status_chatter', RoverSysStatus, queue_size=10)

        # Test Subscription for updating camera_undercarriage
        rospy.Subscriber('camera_2_changer_chatter', Camera2Changer, self.__sub_callback)

        self.msg = RoverSysStatus()
        self.__set_msg_values()

        # Simple Subscriber test boolean -- camera_2_updated.py
        # self.hasCameraUndercarriageChanged = False

    # Callback for test Subscription of camera_2_updater.py -- checks if camera_undercarriage
    # has changed, updates value, and updates bool for run->pub.publish to execute
    def __sub_callback(self, data):
        rospy.loginfo("Camera_undercarriage Status Changed: %d " % data.camera_2_value)
        if self.msg.camera_undercarriage != data.camera_2_value:
            self.msg.camera_undercarriage = data.camera_2_value
            self.hasCameraUndercarriageChanged = True

    # INIT all RoverSysMessage values
    def __set_msg_values(self):
        self.__utc_gps_time()
        self.__bogie_connection_statuses()
        self.__arm_connection_status()
        self.__arm_end_effector_connection_statuses()
        self.__set_cameras()
        self.__sample_containment_connection_status()
        self.__tower_connection_status()
        self.__chassis_pan_tilt_connection_status()
        self.__gps_connection_status()
        self.__jetson_usage_information()
        self.__frsky_controller_connection_status()

    # Pulls the UTC GPS Time (WIP)
    def __utc_gps_time(self):
        self.msg.UTC_GPS_time = 0

    # Pulls bogie connection statuses (WIP)
    def __bogie_connection_statuses(self):
        self.msg.bogie_connection_1 = 0
        self.msg.bogie_connection_2 = 0
        self.msg.bogie_connection_3 = 0

    # Checks arm connection status (WIP)
    def __arm_connection_status(self):
        self.msg.arm_connection_status = 0

    # Checks Arm End Effector Connection Statuses (WIP)
    def __arm_end_effector_connection_statuses(self):
        self.msg.arm_end_effector_connection_statuses = 0

    # Sets the Camera values (zed, undercarriage, chassis, and main_nav
    def __set_cameras(self):
            # Check if camera_zed is found
            self.msg.camera_zed = 1 if os.path.exists(self.system_path_locations[0]) else 0
            # Check if camera_undercarriage is found
            self.msg.camera_undercarriage = 1 if os.path.exists(self.system_path_locations[1]) else 0
            # Check if camera_chassis is found
            self.msg.camera_chassis = 1 if os.path.exists(self.system_path_locations[2]) else 0
            # Check if camera_main_navigation is found
            self.msg.camera_main_navigation = 1 if os.path.exists(self.system_path_locations[3]) else 0

    # Checks Sample Containment Connection Status (WIP)
    def __sample_containment_connection_status(self):
        self.msg.sample_containment_connection_status = 0

    def __tower_connection_status(self):
        self.msg.tower_connection_status = 0

    # Checks Tower Connection Status (WIP)
    def __chassis_pan_tilt_connection_status(self):
        self.msg.chassis_pan_tilt_connection_status = 0

    # Checks GPS Status (WIP)
    def __gps_connection_status(self):
        self.msg.GPS_connection_status = 0

    # Get Jetson Statuses (WIP)
    def __jetson_usage_information(self):
        self.msg.jetson_CPU = 0
        self.msg.jetson_RAM = 0
        self.msg.jetson_EMMC = 0
        self.msg.jetson_NVME_SSD = 0

    # Check FrSky Controller Connection Status (WIP)
    def __frsky_controller_connection_status(self):
        self.msg.FrSky_controller_connection_status = 0

    def run(self):
        r = rospy.Rate(10)
        # rospy.loginfo(self.msg)
        # self.pub.publish(self.msg)
        while not rospy.is_shutdown():
            if (self.msg.camera_zed != os.path.exists(self.system_path_locations[0]) or
                    self.msg.camera_undercarriage != os.path.exists(self.system_path_locations[1]) or
                    self.msg.camera_chassis != os.path.exists(self.system_path_locations[2]) or
                    self.msg.camera_main_navigation != os.path.exists(self.system_path_locations[3])):
                self.__set_cameras()
            rospy.loginfo(self.msg)
            self.pub.publish(self.msg)
            r.sleep()


if __name__ == '__main__':
    system_status = SystemStatuses()
    system_status.run()
