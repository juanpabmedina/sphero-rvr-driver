from pynput import keyboard
import rospy
from geometry_msgs.msg import Vector3

speed = 0
heading = 0
flags = 0
keep_running = True

def on_press(key):
    global speed
    global heading
    global flags
    
    if isinstance(key, keyboard.KeyCode):
        if key.char == 'w':
            if flags == 1:
                speed = 64
            else:
                speed += 64
            flags = 0
        elif key.char == 's':
            if flags == 0:
                speed = 64
            else:
                speed += 64
            flags = 1
        elif key.char == 'a':
            heading -= 10
        elif key.char == 'd':
            heading += 10
    elif key == keyboard.Key.space:
        speed = 0
        flags = 0

    print(speed, heading, flags)

def on_release(key):
    global keep_running
    if key == keyboard.Key.esc:
        keep_running = False
        return False

if __name__ == '__main__':
    rospy.init_node('keyboard_control_node')
    drive_pub = rospy.Publisher('rvr/drive', Vector3, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    try:
        while not rospy.is_shutdown() and keep_running:
            msg = Vector3()
            msg.x = speed
            msg.y = heading
            msg.z = flags
            drive_pub.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        listener.stop()
