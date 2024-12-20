#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import sys
import tty
import termios

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def key_publisher():
    rospy.init_node('rb5_keyInput_node', anonymous=True)
    predefinedPositionpub = rospy.Publisher('/rb5_keyCommand', String, queue_size=10)
    objectTosspub = rospy.Publisher('/rb5_tossCommand', String, queue_size=10)
    rate = rospy.Rate(10) 
    
    rospy.loginfo("1 : Home Position \n  2 : Idle Position \n 3 : Search Position \n 4 : PreToss Position \n 't' : Toss \n 'q' : Quit")

    while not rospy.is_shutdown():
        key = get_key()
        if key in ['1', '2', '3', '4','t']:
            rospy.loginfo(f"Key pressed: {key}")
            predefinedPositionpub.publish(key)
        elif key == 't':
            objectTosspub.publish(key)
        elif key == '\x03':  
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        key_publisher()
    except rospy.ROSInterruptException:
        pass
