#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
import os.path
import psutil
import subprocess
from rover_status.msg import CameraStatuses, BogieStatuses, FrSkyStatus, GPSInfo, MiscStatuses, JetsonInfo
from rover_control.msg import DriveCommandMessage
from std_msgs.msg import Empty


#####################################
# Global Variables
#####################################
DEFAULT_CAMERA_TOPIC_NAME = "camera_status"
DEFAULT_BOGIE_TOPIC_NAME = "bogie_status"
DEFAULT_FRSKY_TOPIC_NAME = "frsky_status"
DEFAULT_GPS_TOPIC_NAME = "gps_status"
DEFAULT_JETSON_TOPIC_NAME = "jetson_status"
DEFAULT_MISC_TOPIC_NAME = "misc_status"

DEFAULT_REQUEST_UPDATE_TOPIC_NAME = "update_requested"


#####################################
# SystemStatuses Class Definition
#####################################
class SystemStatuses:
    def __init__(self):
        # Camera path locations
        self.system_path_locations = [
            '/dev/rover/camera_zed',
            '/dev/rover/camera_undercarriage',
            '/dev/rover/camera_chassis',
            '/dev/rover/camera_main_navigation'
        ]

        # filesystem paths for EMMC [0] and NVME_SSD [1]
        # -- UPDATE [1] FOR JETSON --
        self.file_systems_EMMC_NVMe_SSD = [
            '/',
            '/dev/shm'
        ]

        rospy.init_node('SystemStatuses')

        # Get Topic Names
        self.camera_topic_name = rospy.get_param("~camera_status_topic", DEFAULT_CAMERA_TOPIC_NAME)
        self.bogie_topic_name = rospy.get_param("~bogie_status_topic", DEFAULT_BOGIE_TOPIC_NAME)
        self.frsky_topic_name = rospy.get_param("~frsky_status_topic", DEFAULT_FRSKY_TOPIC_NAME)
        self.gps_topic_name = rospy.get_param("~gps_status_topic", DEFAULT_GPS_TOPIC_NAME)
        self.jetson_topic_name = rospy.get_param("~jetson_status_topic", DEFAULT_JETSON_TOPIC_NAME)
        self.misc_topic_name = rospy.get_param("~misc_status_topic", DEFAULT_MISC_TOPIC_NAME)

        self.request_update_topic_name = rospy.get_param("~request_update_status_topic",
                                                         DEFAULT_REQUEST_UPDATE_TOPIC_NAME)

        # init all publisher functions
        self.pub_camera = rospy.Publisher(self.camera_topic_name, CameraStatuses, queue_size=1)
        self.pub_bogie = rospy.Publisher(self.bogie_topic_name, BogieStatuses, queue_size=1)
        self.pub_FrSky = rospy.Publisher(self.frsky_topic_name, FrSkyStatus, queue_size=1)
        self.pub_GPS = rospy.Publisher(self.gps_topic_name, GPSInfo, queue_size=1)
        self.pub_jetson = rospy.Publisher(self.jetson_topic_name, JetsonInfo, queue_size=1)
        self.pub_Misc = rospy.Publisher(self.misc_topic_name, MiscStatuses, queue_size=1)

        # Subscribers
        self.request_update_subscriber = rospy.Subscriber(self.request_update_topic_name, Empty,
                                                          self.on_update_requested)

        # Manual update variable
        self.manual_update_requested = False

        # init all message variables
        self.camera_msg = CameraStatuses()
        self.bogie_msg = BogieStatuses()
        self.FrSky_msg = FrSkyStatus()
        self.GPS_msg = GPSInfo()
        self.jetson_msg = JetsonInfo()
        self.misc_msg = MiscStatuses()

        # init all message values
        self.__pull_new_message_values()

        # init all previous values
        self.__update_all_previous_values()

    # init all RoverSysMessage values
    def __pull_new_message_values(self):
        self.__set_gps_info()
        self.__set_bogie_connection_statuses()
        self.__set_arm_connection_status()
        self.__set_arm_end_effector_connection_statuses()
        self.__set_cameras()
        self.__set_sample_containment_connection_status()
        self.__set_tower_connection_status()
        self.__set_chassis_pan_tilt_connection_status()
        self.__set_jetson_usage_information()
        self.__set_frsky_controller_connection_status()

    # Pulls the UTC GPS Time (WIP)
    def __set_gps_info(self):
        self.GPS_msg.UTC_GPS_time = 0
        self.GPS_msg.GPS_connection_status = 0

    # Pulls bogie connection statuses (WIP)
    def __set_bogie_connection_statuses(self):
        self.bogie_msg.bogie_connection_1 = 0
        self.bogie_msg.bogie_connection_2 = 0
        self.bogie_msg.bogie_connection_3 = 0

    # Checks arm connection status (WIP)
    def __set_arm_connection_status(self):
        self.misc_msg.arm_connection_status = 0

    # Checks Arm End Effector Connection Statuses (WIP)
    def __set_arm_end_effector_connection_statuses(self):
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
    def __set_sample_containment_connection_status(self):
        self.misc_msg.sample_containment_connection_status = 0

    def __set_tower_connection_status(self):
        self.misc_msg.tower_connection_status = 0

    # Checks Tower Connection Status (WIP)
    def __set_chassis_pan_tilt_connection_status(self):
        self.misc_msg.chassis_pan_tilt_connection_status = 0

    # Get Jetson Statuses (WIP)
    def __set_jetson_usage_information(self):
        self.jetson_msg.jetson_CPU = psutil.cpu_percent()
        mem = psutil.virtual_memory()
        self.jetson_msg.jetson_RAM = mem.percent
        self.jetson_msg.jetson_EMMC = self.__used_percent_fs(self.file_systems_EMMC_NVMe_SSD[0])
        self.jetson_msg.jetson_NVME_SSD = self.__used_percent_fs(self.file_systems_EMMC_NVMe_SSD[1])

        # Temperature
        # This try except causes a bunch of annoying messages, but lets it run on non-jetson devices
        # sets to -1.0 if sensor fails to give it a default value notifying failure to pull
        try:
            sensor_temperatures = subprocess.check_output("sensors | grep temp", shell=True)
            parsed_temps = sensor_temperatures.replace("\xc2\xb0C","").replace("(crit = ","").replace("temp1:","")\
                .replace("\n", "").replace("+", "").split()
            self.jetson_msg.jetson_GPU_temp = float(parsed_temps[4])
        except subprocess.CalledProcessError:
            print('sensors call failed (potential reason: on VM)')
            self.jetson_msg.jetson_GPU_temp = -1.0

    # EMMC and NVMe_SSD used % calculation
    def __used_percent_fs(self, pathname):
        statvfs = os.statvfs(pathname)
        # percentage :: USED:
        #   used amount: blocks - bfree
        #   used%: used_amount / (used_amount + bavail)
        used_available = (statvfs.f_frsize * statvfs.f_blocks / 1024) - (statvfs.f_frsize * statvfs.f_bfree / 1024.0)
        used_percent = used_available / (used_available + (statvfs.f_frsize * statvfs.f_bavail / 1024.0))
        # Round 4 for 2 decimal accuracy
        value = 100 * round(used_percent, 4)
        return value

    # Check FrSky Controller Connection Status (WIP)
    def __set_frsky_controller_connection_status(self):
        rospy.Subscriber('/rover_control/command_control/iris_drive', DriveCommandMessage, self.__frsky_callback)

    def __frsky_callback(self, data):
        self.FrSky_msg.FrSky_controller_connection_status = data.controller_present

    # Used mainly for init, sets all previous values in one go
    def __update_all_previous_values(self):
        self.__set_previous_camera_values()
        self.__set_previous_jetson_values()
        self.__set_previous_frsky_value()
        self.__set_previous_bogie_values()
        self.__set_previous_gps_values()
        self.__set_previous_misc_values()

    def __set_previous_camera_values(self):
        self.previous_camera_zed = self.camera_msg.camera_zed
        self.previous_camera_undercarriage = self.camera_msg.camera_undercarriage
        self.previous_camera_chassis = self.camera_msg.camera_chassis
        self.previous_camera_main_navigation = self.camera_msg.camera_main_navigation

    def __set_previous_jetson_values(self):
        self.previous_jetson_CPU = self.jetson_msg.jetson_CPU
        self.previous_jetson_RAM = self.jetson_msg.jetson_RAM
        self.previous_jetson_EMMC = self.jetson_msg.jetson_EMMC
        self.previous_jetson_NVME_SSD = self.jetson_msg.jetson_NVME_SSD
        self.previous_jetson_GPU_temp = self.jetson_msg.jetson_GPU_temp

    def __set_previous_frsky_value(self):
        self.previous_FrSky_controller_connection_status = self.FrSky_msg.FrSky_controller_connection_status

    def __set_previous_bogie_values(self):
        self.previous_bogie_connection_1 = self.bogie_msg.bogie_connection_1
        self.previous_bogie_connection_2 = self.bogie_msg.bogie_connection_2
        self.previous_bogie_connection_3 = self.bogie_msg.bogie_connection_3

    def __set_previous_gps_values(self):
        self.previous_UTC_GPS_time = self.GPS_msg.UTC_GPS_time
        self.previous_GPS_connection_status = self.GPS_msg.GPS_connection_status

    def __set_previous_misc_values(self):
        self.previous_arm_connection_status = self.misc_msg.arm_connection_status
        self.previous_arm_end_effector_connection_statuses = self.misc_msg.arm_end_effector_connection_statuses
        self.previous_chassis_pan_tilt_connection_status = self.misc_msg.chassis_pan_tilt_connection_status
        self.previous_sample_containment_connection_status = self.misc_msg.sample_containment_connection_status
        self.previous_tower_connection_status = self.misc_msg.tower_connection_status

    def on_update_requested(self,  _):
        self.manual_update_requested = True

    def run(self):
        r = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown():
            # Update all message values
            self.__pull_new_message_values()

            # Camera Check -- if current value is now different from previous value,
            # update the previous value for cameras and publish to listening Subscribers
            if (self.camera_msg.camera_zed != self.previous_camera_zed or
                    self.camera_msg.camera_undercarriage != self.previous_camera_undercarriage or
                    self.camera_msg.camera_chassis != self.previous_camera_chassis or
                    self.camera_msg.camera_main_navigation != self.previous_camera_main_navigation or
                    self.manual_update_requested):
                self.__set_previous_camera_values()
                self.pub_camera.publish(self.camera_msg)

            # Placeholder Jetson Info Check
            if (self.jetson_msg.jetson_CPU != self.previous_jetson_CPU or
                    self.jetson_msg.jetson_RAM != self.previous_jetson_RAM or
                    self.jetson_msg.jetson_EMMC != self.previous_jetson_EMMC or
                    self.jetson_msg.jetson_NVME_SSD != self.previous_jetson_NVME_SSD or
                    self.jetson_msg.jetson_GPU_temp != self.previous_jetson_GPU_temp or
                    self.manual_update_requested):
                self.__set_previous_jetson_values()
                self.pub_jetson.publish(self.jetson_msg)

            # Placeholder FrSky Controller Check
            if (self.FrSky_msg.FrSky_controller_connection_status != self.previous_FrSky_controller_connection_status or
                    self.manual_update_requested):
                self.__set_previous_frsky_value()
                self.pub_FrSky.publish(self.FrSky_msg)

            # Placeholder bogie status check
            if (self.bogie_msg.bogie_connection_1 != self.previous_bogie_connection_1 or
                    self.bogie_msg.bogie_connection_2 != self.previous_bogie_connection_2 or
                    self.bogie_msg.bogie_connection_3 != self.previous_bogie_connection_3 or
                    self.manual_update_requested):
                self.__set_previous_bogie_values()
                self.pub_bogie.publish(self.bogie_msg)

            # Placeholder GPS Information check
            if (self.GPS_msg.UTC_GPS_time != self.previous_UTC_GPS_time or
                    self.GPS_msg.UTC_GPS_time != self.previous_GPS_connection_status or
                    self.manual_update_requested):
                self.__set_previous_gps_values()
                self.pub_GPS.publish(self.GPS_msg)

            # Placeholder Misc Information check
            if (self.misc_msg.arm_connection_status !=
                    self.previous_arm_connection_status or
                    self.misc_msg.arm_end_effector_connection_statuses !=
                    self.previous_arm_end_effector_connection_statuses or
                    self.misc_msg.chassis_pan_tilt_connection_status !=
                    self.previous_chassis_pan_tilt_connection_status or
                    self.misc_msg.sample_containment_connection_status !=
                    self.previous_sample_containment_connection_status or
                    self.misc_msg.tower_connection_status != self.previous_tower_connection_status or
                    self.manual_update_requested):
                self.__set_previous_misc_values()
                self.pub_Misc.publish(self.misc_msg)

            if self.manual_update_requested:
                self.manual_update_requested = False

            r.sleep()


if __name__ == '__main__':
    system_status = SystemStatuses()
    system_status.run()
