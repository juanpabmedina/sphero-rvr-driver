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


rvr = SpheroRvrObserver()
# class SpheroRVR():

#     def __init__(self) -> None:

#         # init ROS node
#         rospy.init_node("sphero_RVR_driver_node")

#     def create_ros_subscribers(self) -> None:

#         # Velocity 
#         rospy.Subscriber('sphero/leds', ColorRGBA, 
#                          self.cmd_leds_callback, queue_size=1)
    
#     def cmd_leds_callback(self, msg) -> None:
#             rvr.set_all_leds(
#             led_group=RvrLedGroups.headlight_right.value,   # 0xe00
#             led_brightness_values=[msg.r, msg.g, msg.b]
#         )

def leds_callback(msg) -> None:
    rvr.set_all_leds(
    led_group=RvrLedGroups.headlight_right.value,   # 0xe00
    led_brightness_values=[msg.r, msg.g, msg.b]
)
        
if __name__ == '__main__':
    try: 
        rospy.init_node("sphero_RVR_driver_node")
        rospy.Subscriber('sphero/leds', ColorRGBA, leds_callback, queue_size=1)

        rate=rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()
    except:
        pass