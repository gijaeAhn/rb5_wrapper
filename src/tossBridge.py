#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from queue import Queue

class TossHandler:
    def __init__(self):
        rospy.init_node('rb5_tossBridge_node', anonymous=True)
        
        self.object_queue = Queue()
        
        rospy.Subscriber('/rb5_objectPosition', TransformStamped, self.object_position_callback)
        rospy.Subscriber('/rb5_tossCommand', String, self.toss_command_callback)
        
        self.throwing_planner_pub = rospy.Publisher('/throwing_planner_command', TransformStamped, queue_size=10)
        
        rospy.loginfo("Toss Handler Node Initialized...")
    
    def object_position_callback(self, msg):
        rospy.loginfo("Received Object Position, adding to queue...")
        self.object_queue.put(msg)
    
    def toss_command_callback(self, msg):
        if msg.data == 't':
            rospy.loginfo("Received Toss Command.")
            if not self.object_queue.empty():
                object_position = self.object_queue.get()
                rospy.loginfo("Sending Object Position to Throwing Planner...")
                
                self.throwing_planner_pub.publish(object_position)
            else:
                rospy.logwarn("No object position available in the queue!")

if __name__ == '__main__':
    try:
        toss_handler = TossHandler()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Toss Handler Node Shutdown.")
