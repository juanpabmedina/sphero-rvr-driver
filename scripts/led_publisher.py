#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Vector3, Twist

        
if __name__ == '__main__':
    try: 
        rospy.init_node("sphero_RVR_driver_publisher")
        pub_led = rospy.Publisher('/robot/cmd_led/', Vector3, queue_size=10)
        rate=rospy.Rate(30)

        while not rospy.is_shutdown():
            pub_led.publish([255, 0, 0])
            rate.sleep()
    except:
        pass