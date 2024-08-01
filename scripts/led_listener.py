#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

import sys
import os
import time
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import asyncio

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import SerialAsyncDal


rvr = SpheroRvrObserver()

def right_led_callback(data):
    rospy.loginfo(data)
    rvr.set_all_leds(
        led_group=RvrLedGroups.headlight_right.value,   # 0xe00
        led_brightness_values=[round(data.x),round(data.y),round(data.z)]
    )

def left_led_callback(data):
    rospy.loginfo(data)
    rvr.set_all_leds(
        led_group=RvrLedGroups.headlight_left.value,   # 0xe00
        led_brightness_values=[round(data.x),round(data.y),round(data.z)]
    )


    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    try:
        rospy.init_node('rvr_led_listener_node')

        rospy.Subscriber("/robot/teleoperation/mouse_pose/", Vector3, right_led_callback)
        rospy.Subscriber("/robot/teleoperation/mouse_pose/", Vector3, left_led_callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
        rate=rospy.Rate(30)

        while not rospy.is_shutdown():
            rate.sleep()

    except KeyboardInterrupt:
        print('\nProgram terminated with keyboard interrupt.')

    finally:
        rvr.close()


if __name__ == '__main__':
    rvr.wake()

    # Give RVR time to wake up
    time.sleep(2)
    listener()
