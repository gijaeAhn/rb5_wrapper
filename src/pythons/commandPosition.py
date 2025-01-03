#!/usr/bin/env python3

import rospy

import tf2_ros
from tf_conversions import transformations

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rb5_wrapper.srv import *

import numpy as np

# Set Predefined Matrices 
# 1 : Init Position
# 2 : Idle Position
# 3 : Search Position
# 4 : Pre toss Position <<- This will be dynamically changed

predefinedMatrices = {
    1: np.array([[1, 0, 0, 0.2],
                 [0, 0, -1, -0.1],
                 [0, 1, 0, 0.4],
                 [0, 0, 0, 1.0]]),

    2: np.array([[0, 0, 1, 0.5],
                 [1, 0, 0, 0.1],
                 [0, 1, 0, 0.3],
                 [0, 0, 0, 1.0]]),

    3: np.array([[1, 0, 0, 0.2],
                 [0, 1, 0, 0.0],
                 [0, 0, 1, 0.3],
                 [0, 0, 0, 1.0]]),
    
    4: None
}

def matrixToQuaternion(matrix):
    """Convert 4x4 transformation matrix to quaternion."""
    # quaternion_from_matrix returns (x, y, z, w)
    return transformations.quaternion_from_matrix(matrix)

def matrixToMsg(matirx,frame = "world",child_frame = "rb5_target"):

    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = frame
    t.child_frame_id = child_frame

    translation = matrix[:3, 3]
    quaternion = matrixToQuaternion(matrix)
    
    t.transform.translation.x = translation[0] * 1000
    t.transform.translation.y = translation[1] * 1000
    t.transform.translation.z = translation[2] * 1000
    
    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

def commandCallback(msg, pub):
   try:
       data = msg.data
       if isinstance(data, int):
           handlePosition(data, pub)
       elif isinstance(data, str):
           handleCommand(data)
       else:
           rospy.logwarn("Invalid data type received. Expected int or str.")
   except Exception as e:
       rospy.logwarn(f"Error processing command: {e}")

def handlePosition(idx, pub):
   if idx not in predefinedMatrices:
       rospy.logwarn(f"Invalid index {idx}: not in predefined matrices")
       return
       
   matrix = predefinedMatrices[idx]
   
   t = matrixToMsg(matirx)
   pub.publish(t)
   rospy.loginfo(f"Published Position {idx}: Target frame updated")

def handleCommand(cmd):
   if cmd == 'toss':
       tossFunction()
   elif cmd == 'search':
       searchFunction()
   else:
       rospy.logwarn(f"Unknown command: {cmd}")

def tossFunction(pub):
   if not predefinedMatrices:
       rospy.logwarn("No predefined matrices available")
       return
       
   if predefinedMatrices[3] is None:
       rospy.logwarn("No target position in slot 3")
       return

   try:
       target_matrix = predefinedMatrices[3]
       pre_toss_matrix = np.copy(target_matrix)
       pre_toss_matrix[2,3] += 0.3  # Move up 30cm in Z
       
       # Move to pre-toss position
       t_pre = matrixToMsg(pre_toss_matrix)
       
       pub.publish(t_pre)
       rospy.sleep(2)
       
       # Execute toss motion
       t_toss = matrixToMsg(target_matrix)
       
       # Clear target position
       predefinedMatrices[3] = None
       
   except Exception as e:
       rospy.logerr(f"Toss execution failed: {e}")

def searchFunction():
   try:
       rospy.wait_for_service('search_aruco', timeout=5.0)
       search_client = rospy.ServiceProxy('search_aruco', SearchAruco)
       response = search_client()
       
       if response.success:
           camera_to_target = response.transform
           
           # Will vary depends on HardWare Specification
           ee_to_camera = np.array([
               [0, -1, 0, 0.05],
               [1, 0, 0, 0],
               [0, 0, 1, 0.03],
               [0, 0, 0, 1]
           ])
           
           # Static search position matrix
           world_to_ee = predefinedMatrices[2]
           world_to_target = np.dot(world_to_ee, np.dot(ee_to_camera, camera_to_target))

           # Store the target location 
           predefinedMatrices[3] = world_to_target

   except Exception as e:
       rospy.logerr(f"Search transform failed: {e}")
       return None

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