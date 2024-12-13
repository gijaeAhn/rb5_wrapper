#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
import tf_conversions

# Predefined fixed matrices for positions 1 and 2
predefined_matrices = {
    1: np.array([[1, 0, 0, 0.2],
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),

    2: np.array([[1, 0, 0, 0.5],
                 [0, 1, 0, 0.1],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]])
}

# Dynamic positions: source and target frames
dynamic_frames = {
    3: {"source": "source_frame_3", "target": "world"},
    4: {"source": "source_frame_4", "target": "world"}
}

def matrix_to_quaternion(matrix):
    """Convert 4x4 transformation matrix to quaternion."""
    return tf_conversions.transformations.quaternion_from_matrix(matrix)

def get_dynamic_transform(buffer, idx):
    """Fetch dynamic transform for positions 3 or 4."""
    try:
        source = dynamic_frames[idx]["source"]
        target = dynamic_frames[idx]["target"]
        return buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(0.5))
    except (tf2_ros.LookupException, tf2_ros.ExtrapolationException, tf2_ros.TransformException):
        return None

def publish_position(pub, idx, tf_buffer=None):
    """Publish fixed or dynamic position."""
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "rb5_target"

    if idx in [1, 2]:  # Fixed positions
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

    elif idx in [3, 4]:  # Dynamic positions
        transform = get_dynamic_transform(tf_buffer, idx)
        if transform:
            t.transform.translation.x = transform.transform.translation.x
            t.transform.translation.y = transform.transform.translation.y
            t.transform.translation.z = transform.transform.translation.z

            t.transform.rotation.x = transform.transform.rotation.x
            t.transform.rotation.y = transform.transform.rotation.y
            t.transform.rotation.z = transform.transform.rotation.z
            t.transform.rotation.w = transform.transform.rotation.w
        else:
            rospy.logwarn(f"Dynamic position {idx} is not available yet.")
            return

    # Publish the transform
    pub.publish(t)
    rospy.loginfo(f"Published Position {idx}: Target frame updated.")

def main():
    rospy.init_node('key_input_command_tf_publisher', anonymous=True)
    pub = rospy.Publisher('/rb5_target', TransformStamped, queue_size=10)

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    print("Press 1, 2 (Fixed Positions) or 3, 4 (Dynamic Positions) to publish. Press 'q' to quit.")
    
    while not rospy.is_shutdown():
        user_input = input("Enter a key (1, 2, 3, 4) to publish position: ").strip()
        if user_input.lower() == 'q':
            print("Exiting...")
            break

        try:
            idx = int(user_input)
            if idx in [1, 2, 3, 4]:
                publish_position(pub, idx, tf_buffer)
            else:
                print("Invalid input")
        except ValueError:
            print("Invalid input")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
