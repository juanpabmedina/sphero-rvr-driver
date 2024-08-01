#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

def talker():
    pub = rospy.Publisher('led_color', Vector3, queue_size=10)
    rospy.init_node('rvr_led_talker_node')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = Vector3(1,2,3)
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
