#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf_conversions
import threading
import queue

# Input queue for handling user input
input_queue = queue.Queue()

# Predefined transformation matrices
predefined_matrices = {
    1: np.array([[1, 0, 0, 0.2], 
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),

    2: np.array([[1, 0, 0, 0.5],  
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.7],
                 [0, 0, 0, 1.0]]),

    3: np.array([[1, 0, 0, 0.4], 
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.6],
                 [0, 0, 0, 1.0]]),
                 
    4: np.array([[1, 0, 0, 0.1], 
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.2],
                 [0, 0, 0, 1.0]])
}

# Convert a transformation matrix to quaternion
def matrix_to_quaternion(matrix):
    quaternion = tf_conversions.transformations.quaternion_from_matrix(matrix)
    return quaternion

# Input listener thread for user input
def input_listener():
    while not rospy.is_shutdown():
        try:
            user_input = int(input("Enter a number (1-4): "))
            if user_input in predefined_matrices:
                input_queue.put(user_input)
            else:
                print("Invalid input")
        except ValueError:
            print("Invalid input")

# Publish geometry_msgs/TransformStamped messages
def publish_predefined_positions():
    rospy.init_node('predefined_tf_publisher', anonymous=True)
    pub = rospy.Publisher('/rb5_target', TransformStamped, queue_size=10)
    rate = rospy.Rate(10)

    print("Ready to accept input. Enter 1, 2, 3, 4")

    while not rospy.is_shutdown():
        if not input_queue.empty():
            user_input = input_queue.get()

            matrix = predefined_matrices[user_input]
            translation = matrix[:3, 3]
            quaternion = matrix_to_quaternion(matrix)

            # Create and populate TransformStamped message
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "world"
            t.child_frame_id = "rb5_target"

            t.transform.translation.x = translation[0]
            t.transform.translation.y = translation[1]
            t.transform.translation.z = translation[2]

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

            # Publish the message
            pub.publish(t)
            rospy.loginfo(f"Published Position {user_input}: Translation[{translation[0]}, {translation[1]}, {translation[2]}] "
                          f"Quaternion[{quaternion[0]}, {quaternion[1]}, {quaternion[2]}, {quaternion[3]}]")

        rate.sleep()

if __name__ == "__main__":
    try:
        input_thread = threading.Thread(target=input_listener)
        input_thread.daemon = True
        input_thread.start()

        publish_predefined_positions()
    except rospy.ROSInterruptException:
        pass