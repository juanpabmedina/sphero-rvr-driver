#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

def talker():
    led_color_right = rospy.Publisher('led_color/right', Vector3, queue_size=10)
    led_color_left = rospy.Publisher('led_color/left', Vector3, queue_size=10)
    rospy.init_node('rvr_led_talker_node')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        right_color = Vector3(255,0,0)
        left_color = Vector3(0,0,255)
        led_color_right.publish(right_color)
        led_color_left.publish(left_color)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
