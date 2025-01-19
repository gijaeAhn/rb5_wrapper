import sys
import os
import threading

import rospy
from std_msgs.msg import Bool

home_dir = os.path.expanduser('~')
sys.path.append(os.path.join(home_dir, 'Desktop/catkin_ws/src/tossing/src/end_effector'))


import GRIPPER.Gripper
from GRIPPER import Gripper
from GRIPPER import mainGripper

gripper_enabled = False
state_changed = False

# from lobbing import Lobbing

def state_callback(msg):
    global gripper_enabled
    global state_changed
    gripper_enabled = msg.data
    state_changed = True 
    rospy.loginfo(f"Received state change: {gripper_enabled}")

def tossing():
        mainGripper.eeTossing()

def searching():
        mainGripper.eeSearching()

def main():
    global gripper_enabled
    global state_changed
    rospy.init_node('ee_control_node', anonymous=True)
    rospy.Subscriber('/ee_enable', Bool, state_callback)

    rospy.loginfo("Waiting for control messages...")
    rate = rospy.Rate(100)  
    
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        if state_changed:
            rospy.loginfo("State changed, pausing 1Sec")
            rospy.sleep(1)
            state_changed = False 

        if gripper_enabled:
            rospy.loginfo("Tossing!!")
            tossing()
        else:
            rospy.loginfo("Searching!!")
            searching()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print("ROS node interrupted.")