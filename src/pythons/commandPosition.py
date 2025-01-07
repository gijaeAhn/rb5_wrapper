#!/usr/bin/env python3
import rospy

import tf2_ros
from tf_conversions import transformations

from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from rb5_ros_wrapper.srv import *

import numpy as np

# Set Predefined Matrices 
# 1 : Init Position
# 2 : Idle Position
# 3 : Search Position
# 4 : Pre toss Position <<- This will be dynamically changed

targetSpace = np.array([[1, 0,  0,  0.2],
                        [0, 0, -1,  0.0],
                        [0, 1,  0,  0.4],
                        [0, 0,  0,  1.0]])

# X = -11.5 mm
# Y =  31   mm
# Z = Unknown
eeToRgbCenter = np.array([[1, 0,  0,  -0.0115],
                          [0, 1,  0,   0.031 ],
                          [0, 0,  1,   0.4   ],
                          [0, 0,  0,  1.0]])
searchPosition = np.array([[1, 0,  0,   0.2],
                           [0, 1,  0,   0.0],
                           [0, 0,  1,   0.4],
                           [0, 0,  0,  1.0]])

predefinedMatrices = {
    1: np.array([[1, 0, 0, 0.2],
                 [0, 0, -1, -0.1],
                 [0, 1, 0, 0.4],
                 [0, 0, 0, 1.0]]),

    2: np.array([[0, 0, 1, 0.5],
                 [1, 0, 0, 0.1],
                 [0, 1, 0, 0.3],
                 [0, 0, 0, 1.0]]),

    # Search Position should be at the center of Target Space
    3: np.dot(searchPosition, eeToRgbCenter.transpose()),
    
    4: None
}

def matrixToQuaternion(matrix):
    """Convert 4x4 transformation matrix to quaternion."""
    # quaternion_from_matrix returns (x, y, z, w)
    return transformations.quaternion_from_matrix(matrix)

def matrixToMsg(matrix,frame = "world",child_frame = "rb5_target"):

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

    return t

def transformToMatrix(msg):
   matrix = np.eye(4)
   
   # Translation
   matrix[:3, 3] = [
       msg.transform.translation.x,
       msg.transform.translation.y,
       msg.transform.translation.z
   ]
   
   # Rotation from quaternion
   quat = [
       msg.transform.rotation.x,
       msg.transform.rotation.y,
       msg.transform.rotation.z,
       msg.transform.rotation.w
   ]
   matrix[:3, :3] = tf.transformations.quaternion_matrix(quat)[:3, :3]
   
   return matrix

def commandCallback(msg, pub):
   try:
       data = msg.data
       print("Data : ",data)
       if data.isdigit():
           print("Debug 1")
           idx = int(data)
           handlePosition(idx, pub)
           print("Debug 1-1")

       else :
            print("Debug 2")
            if data == 's':
                command = 'search'
            elif data == 't':
                command = 'toss'
            handleCommand(command,pub)
            print("Debug 2-1")

   except Exception as e:
       rospy.logwarn(f"Error processing command: {e}")

def handlePosition(idx, pub):
   if idx-1 not in predefinedMatrices:
       rospy.logwarn(f"Invalid index {idx-1}: not in predefined matrices")
       return
       
   matrix = predefinedMatrices[idx-1]
   
   t = matrixToMsg(matrix)
   pub.publish(t)
   rospy.loginfo(f"Published Position {idx-1}: Target frame updated")

def handleCommand(cmd,pub):
   if cmd == 'toss':
       tossFunction(pub)
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
           rospy.loginfo(f"Target Searched : Update Target Frame")

           camera_to_target = response.transform
           camera_to_target_np = transformToMatrix(camera_to_target)
           world_to_target = np.dot(searchPosition,camera_to_target_np)

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