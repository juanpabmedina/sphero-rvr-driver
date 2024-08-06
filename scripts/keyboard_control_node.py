#!/usr/bin/env python
# license removed for brevity
import rospy
import asyncio
import signal

from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Twist

import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))
from helper_keyboard_input import KeyboardHelper

key_helper = KeyboardHelper()
current_key_code = -1
driving_keys = [119, 97, 115, 100, 32]
speed = 0
heading = 0
flags = 0

loop = asyncio.get_event_loop()

def keycode_callback(keycode):
    global current_key_code
    current_key_code = keycode
    print("Key code updated: ", str(current_key_code))

async def main():
    global current_key_code
    global speed
    global heading
    global flags

    while not rospy.is_shutdown():
        if current_key_code == 119:  # W
            if flags == 1:
                speed = 64
            else:
                speed += 64
            flags = 0
        elif current_key_code == 97:  # A
            heading -= 10
        elif current_key_code == 115:  # S
            if flags == 0:
                speed = 64
            else:
                speed += 64
            flags = 1
        elif current_key_code == 100:  # D
            heading += 10
        elif current_key_code == 32:  # SPACE
            speed = 0
            flags = 0

        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255

        if heading > 359:
            heading -= 359
        elif heading < 0:
            heading = 359 + heading

        current_key_code = -1
        rospy.loginfo(Vector3(speed, heading, flags))
        drive_pub.publish(Vector3(speed, heading, flags))
        await asyncio.sleep(0.1)

def run_loop():
    global loop
    global key_helper
    key_helper.set_callback(keycode_callback)
    loop.run_until_complete(
        asyncio.gather(
            main()
        )
    )

def signal_handler(sig, frame):
    print("Shutting down...")
    key_helper.end_get_key_continuous()
    rospy.signal_shutdown("Keyboard Interrupt")
    exit(0)

if __name__ == '__main__':
    drive_pub = rospy.Publisher('rvr/drive', Vector3, queue_size=10)
    rospy.init_node('keyboard_control_node')
    signal.signal(signal.SIGINT, signal_handler)
    loop.run_in_executor(None, key_helper.get_key_continuous)
    try:
        run_loop()
    except KeyboardInterrupt:
        print("Keyboard Interrupt...")
    finally:
        print("Press any key to exit.")
