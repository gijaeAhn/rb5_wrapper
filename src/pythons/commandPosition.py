#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
from tf_conversions import transformations

# Set Predefined Matrices 
# 1 : Init Position
# 2 : Idle Position
# 3 : Search Position
# 4 : Pre toss Position

predefined_matrices = {
    1: np.array([[1, 0, 0, 0.2],
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),

    2: np.array([[1, 0, 0, 0.5],
                 [0, 1, 0, 0.1],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),

    3: np.array([[1, 0, 0, 0.2],
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),
    
    4: np.array([[1, 0, 0, 0.2],
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]])
}


def matrix_to_quaternion(matrix):
    """Convert 4x4 transformation matrix to quaternion."""
    # quaternion_from_matrix returns (x, y, z, w)
    return transformations.quaternion_from_matrix(matrix)


def commandCallback(msg, pub):

    try:
        idx = int(msg.data)
    except ValueError:
        rospy.logwarn("Received non-integer command. Ignoring.")
        return

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "rb5_target"

    if idx in predefined_matrices:
        matrix = predefined_matrices[idx]
        translation = matrix[:3, 3]
        quaternion = matrix_to_quaternion(matrix)

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Publish the transform
        pub.publish(t)
        rospy.loginfo(f"Published Position {idx}: Target frame updated.")
    else:
        rospy.logwarn("Received index not in predefined matrices. Ignoring.")


def main():
    rospy.init_node('rb5_command_node', anonymous=True)
    pub = rospy.Publisher('/rb5_target', TransformStamped, queue_size=10)
    sub = rospy.Subscriber('/rb5_keyCommand', String, commandCallback, pub)

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
