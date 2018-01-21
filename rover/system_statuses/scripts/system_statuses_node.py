#!/usr/bin/env python
import rospy
import os.path
from system_statuses.msg import CameraStatuses, BogieStatuses, FrSkyStatus, GPSInfo, MiscStatuses, JetsonInfo



class SystemStatuses:

    def __init__(self):
        # Camera path locations
        self.system_path_locations = [
            '/dev/rover/camera_zed',
            '/dev/rover/camera_undercarriage',
            '/dev/rover/camera_chassis',
            '/dev/rover/camera_main_navigation'
        ]

        rospy.init_node('SystemStatuses')

        # init all publisher functions
        self.pub_camera = rospy.Publisher('camera_system_status_chatter', CameraStatuses, queue_size=10)
        self.pub_bogie = rospy.Publisher('bogie_system_status_chatter', BogieStatuses, queue_size=10)
        self.pub_FrSky = rospy.Publisher('FrSky_system_status_chatter', FrSkyStatus, queue_size=10)
        self.pub_GPS = rospy.Publisher('GPS_system_status_chatter', GPSInfo, queue_size=10)
        self.pub_jetson = rospy.Publisher('jetson_system_status_chatter', JetsonInfo, queue_size=10)
        self.pub_Misc = rospy.Publisher('misc_system_status_chatter', MiscStatuses, queue_size=10)

        # init all message variables
        self.camera_msg = CameraStatuses()
        self.bogie_msg = BogieStatuses()
        self.FrSky_msg = FrSkyStatus()
        self.GPS_msg = GPSInfo()
        self.jetson_msg = JetsonInfo()
        self.misc_msg = MiscStatuses()
        self.__set_msg_values()

    # INIT all RoverSysMessage values
    def __set_msg_values(self):
        self.__gps_update()
        self.__bogie_connection_statuses()
        self.__arm_connection_status()
        self.__arm_end_effector_connection_statuses()
        self.__set_cameras()
        self.__sample_containment_connection_status()
        self.__tower_connection_status()
        self.__chassis_pan_tilt_connection_status()
        self.__jetson_usage_information()
        self.__frsky_controller_connection_status()

    # Pulls the UTC GPS Time (WIP)
    def __gps_update(self):
        self.GPS_msg.UTC_GPS_time = 0
        self.GPS_msg.GPS_connection_status = 0

    # Pulls bogie connection statuses (WIP)
    def __bogie_connection_statuses(self):
        self.bogie_msg.bogie_connection_1 = 0
        self.bogie_msg.bogie_connection_2 = 0
        self.bogie_msg.bogie_connection_3 = 0

    # Checks arm connection status (WIP)
    def __arm_connection_status(self):
        self.misc_msg.arm_connection_status = 0

    # Checks Arm End Effector Connection Statuses (WIP)
    def __arm_end_effector_connection_statuses(self):
        self.misc_msg.arm_end_effector_connection_statuses = 0

    # Sets the Camera values (zed, undercarriage, chassis, and main_nav
    def __set_cameras(self):
            # Check if camera_zed is found
            self.camera_msg.camera_zed = 1 if os.path.exists(self.system_path_locations[0]) else 0
            # Check if camera_undercarriage is found
            self.camera_msg.camera_undercarriage = 1 if os.path.exists(self.system_path_locations[1]) else 0
            # Check if camera_chassis is found
            self.camera_msg.camera_chassis = 1 if os.path.exists(self.system_path_locations[2]) else 0
            # Check if camera_main_navigation is found
            self.camera_msg.camera_main_navigation = 1 if os.path.exists(self.system_path_locations[3]) else 0

    # Checks Sample Containment Connection Status (WIP)
    def __sample_containment_connection_status(self):
        self.misc_msg.sample_containment_connection_status = 0

    def __tower_connection_status(self):
        self.misc_msg.tower_connection_status = 0

    # Checks Tower Connection Status (WIP)
    def __chassis_pan_tilt_connection_status(self):
        self.misc_msg.chassis_pan_tilt_connection_status = 0

    # Get Jetson Statuses (WIP)
    def __jetson_usage_information(self):
        self.jetson_msg.jetson_CPU = 0
        self.jetson_msg.jetson_RAM = 0
        self.jetson_msg.jetson_EMMC = 0
        self.jetson_msg.jetson_NVME_SSD = 0

    # Check FrSky Controller Connection Status (WIP)
    def __frsky_controller_connection_status(self):
        self.FrSky_msg.FrSky_controller_connection_status = 0

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Camera Check & update if change
            if (self.camera_msg.camera_zed != os.path.exists(self.system_path_locations[0]) or
                    self.camera_msg.camera_undercarriage != os.path.exists(self.system_path_locations[1]) or
                    self.camera_msg.camera_chassis != os.path.exists(self.system_path_locations[2]) or
                    self.camera_msg.camera_main_navigation != os.path.exists(self.system_path_locations[3])):
                self.__set_cameras()
                self.pub_camera.publish(self.camera_msg)

            # Placeholder Jetson Info Check
            if (self.jetson_msg.jetson_CPU != 0 or
                    self.jetson_msg.jetson_RAM != 0 or
                    self.jetson_msg.jetson_EMMC != 0 or
                    self.jetson_msg.jetson_NVME_SSD != 0):
                self.__jetson_usage_information()
                self.pub_jetson.publish(self.jetson_msg)

            # Placeholder FrSky Controller Check
            if self.FrSky_msg.FrSky_controller_connection_status != 0:
                self.__frsky_controller_connection_status()
                self.pub_FrSky.publish(self.FrSky_msg)

            # Placeholder bogie status check
            if (self.bogie_msg.bogie_connection_1 != 0 or
                    self.bogie_msg.bogie_connection_2 != 0 or
                    self.bogie_msg.bogie_connection_3 != 0):
                self.__bogie_connection_statuses()
                self.pub_bogie.publish(self.bogie_msg)

            # Placeholder GPS Information check
            if (self.GPS_msg.UTC_GPS_time != 0 or self.GPS_msg.UTC_GPS_time != 0):
                self.__gps_update()
                self.pub_GPS.publish(self.GPS_msg)

            # rospy.loginfo(self.msg)
            r.sleep()


if __name__ == '__main__':
    system_status = SystemStatuses()
    system_status.run()
