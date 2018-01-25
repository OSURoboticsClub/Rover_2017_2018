import rospy

from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64

if __name__ == "__main__":
    rospy.init_node("imu_test")
    
    
    orientation = Quaternion()
    orientation_covarience = Float64()
    
    angular_velocity = Vector3()
    angular_velocity_covarience = Float64()
    
    linear_acceleration = Vector3()
    linear_acceleration_covarience = Float64()