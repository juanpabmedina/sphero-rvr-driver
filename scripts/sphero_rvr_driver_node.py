#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String, ColorRGBA, Bool
from geometry_msgs.msg import Vector3, Twist
import time

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../')))

import asyncio

from sphero_sdk import SpheroRvrObserver
from sphero_sdk import Colors
from sphero_sdk import RvrLedGroups
from sphero_sdk import SerialAsyncDal

import signal

class SpheroRVR():

    PERIOD_PUBLISH_CALLBACK: float = 0.1
    PERIOD_CONTROL_CALLBACK: float = 0.25

    def __init__(self) -> None:

        # Set up ROS node and signal handler
        rospy.init_node("sphero_RVR_driver_node")
        signal.signal(signal.SIGINT, self.signal_handler)

        # RVR SDK
        self.rvr = SpheroRvrObserver()
        # RVR wake up
        self.rvr.wake()
        # Give RVR time to wake up
        time.sleep(2)
        self.setup_rvr_parameters()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Shutting down...")
        self.rvr.close()
        rospy.signal_shutdown("Keyboard Interrupt")

    def setup_rvr_parameters(self) -> None:
        self.create_ros_publishers()
        self.create_ros_subscribers()

        init_rvr_drive = [0, 0, 0]
        self.drive_rvr = Vector3(init_rvr_drive[0], init_rvr_drive[1], init_rvr_drive[2])

        # Publishing timer
        rospy.Timer(
            rospy.Duration(self.PERIOD_PUBLISH_CALLBACK), self.publisher_callback)
        rospy.Timer(
            rospy.Duration(self.PERIOD_CONTROL_CALLBACK), self.control_loop_callback)

    def create_ros_subscribers(self) -> None:
        # Drive robot (velocity, heading, flags)
        rospy.Subscriber('rvr/drive', Vector3, self.drive_callback, queue_size=1)
        # Leds
        rospy.Subscriber('rvr/right_led', Vector3, self.right_led_callback, queue_size=1)
        rospy.Subscriber('rvr/left_led', Vector3, self.left_led_callback, queue_size=1)

    def drive_callback(self, data):
        self.drive_rvr = Vector3(round(data.x), round(data.y), round(data.z))

    def right_led_callback(self, data) -> None:
        self.rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_right.value,
            led_brightness_values=[round(data.x), round(data.y), round(data.z)]
        )

    def left_led_callback(self, data) -> None:
        self.rvr.set_all_leds(
            led_group=RvrLedGroups.headlight_left.value,
            led_brightness_values=[round(data.x), round(data.y), round(data.z)]
        )

    def create_ros_publishers(self) -> None:
        self.pub_battery_percentage = rospy.Publisher('rvr/battery', Bool, queue_size=1)

    def publish_battery_percentage(self):
        battery_percentage_msg = self.rvr.get_battery_percentage()
        self.pub_battery_percentage.publish(battery_percentage_msg)

    def publisher_callback(self, event=None):
        self.publish_battery_percentage()

    def set_drive_rvr(self):
        self.rvr.drive_with_heading(self.drive_rvr.x, self.drive_rvr.y, self.drive_rvr.y)

    def control_loop_callback(self, event=None):
        self.set_drive_rvr()

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("ROS interrupted")
        finally:
            self.rvr.close()

if __name__ == "__main__":
    try:
        sphero_rvr = SpheroRVR()
        sphero_rvr.run()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupted")
    finally:
        rospy.loginfo("Exiting program")
