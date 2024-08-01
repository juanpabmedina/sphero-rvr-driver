#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, ColorRGBA
from geometry_msgs.msg import Vector3, Twist

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import asyncio

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import SerialAsyncDal

class SpheroRVR():

    def __init__(self) -> None:

        # init ROS node
        rospy.init_node("sphero_RVR_driver_node")
        self.rvr = SpheroRvrObserver()

    def create_ros_subscribers(self) -> None:
        # Drive robot (velocity, heading, flags) 
        rospy.Subscriber('rvr/drive', Vector3, self.drive_callback, queue_size=1)
        # Leds
        rospy.Subscriber('rvr/right_led', Twist, self.right_led_callback, queue_size=1)
        rospy.Subscriber('rvr/left_led', Twist, self.left_led_callback, queue_size=1)        

    def create_ros_publishers(self) -> None:
        self.pub_battery_percentage = rospy.Publisher('rvr/battery', bool, queue_size=1)

    def drive_callback(self, data):
        # rospy.loginfo(data)
        self.rvr.drive_with_heading(round(data.x), round(data.y), round(data.z))    

    def right_led_callback(self,data):
        # rospy.loginfo(data)
        self.rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_right.value,   # 0xe00
            led_brightness_values=[round(data.x),round(data.y),round(data.z)]
        )

    def left_led_callback(self,data):
        rospy.loginfo(data)
        self.rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_left.value,   # 0xe00
            led_brightness_values=[round(data.x),round(data.y),round(data.z)]
        )

    def publish_battery_percentage(self):
        battery_percentage_msg = self.rvr.get_battery_percentage()
        self.pub_battery_percentage.publish(battery_percentage_msg)

        
if __name__ == "__main__":

    try:
        sphero_rvr = SpheroRVR()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Keyboard interrupted")
        exit()