from sphero_rvr_driver_node import SpheroRVR

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

rvr = SpheroRVR()

def talker():
    rvr.create_ros_publishers
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rvr.publish_battery_percentage
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass