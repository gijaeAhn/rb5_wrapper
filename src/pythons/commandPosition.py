#!/usr/bin/env python3
import rospy

import tf2_ros
from tf_conversions import transformations
import tf.transformations as tft

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from rb5_ros_wrapper.srv import *

import numpy as np

# Set Predefined Matrices 
# 1 : Init Position
# 2 : Search Position
# 3 : 
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
searchPosition = np.array([[1, 0,  0,   0.4],
                           [0, -1,  0,   0.0],
                           [0, 0,  -1,   0.4],
                           [0, 0,  0,  1.0]])

predefinedMatrices = {
    1: np.array([[1, 0, 0, 0.2],
                 [0, -1, 0, -0.1],
                 [0, 0, -1, 0.4],
                 [0, 0, 0, 1.0]]),

    2: np.array([[1, 0, 0, 0.4],
                 [0, -1, 0, 0.0],
                 [0, 0, -1, 0.4],
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
       msg.translation.x,
       msg.translation.y,
       msg.translation.z
   ]
   
   # Rotation from quaternion
   quat = [
       msg.rotation.x,
       msg.rotation.y,
       msg.rotation.z,
       msg.rotation.w
   ]

   matrix_4x4 = tft.quaternion_matrix(quat)
   matrix[:3, :3] = matrix_4x4[:3, :3]
   
   return matrix

def commandCallback(msg, args):
   pub, eePub = args
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
            handleCommand(command,pub,eePub)
            print("Debug 2-1")

   except Exception as e:
       rospy.logwarn(f"Error processing command: {e}")

def handlePosition(idx, pub):
   if idx not in predefinedMatrices:
       rospy.logwarn(f"Invalid index {idx}: not in predefined matrices")
       return
       
   matrix = predefinedMatrices[idx]
   
   t = matrixToMsg(matrix)
   pub.publish(t)
   rospy.loginfo(f"Published Position {idx}: Target frame updated")

def handleCommand(cmd,pub,eePub):
   if cmd == 'toss':
       tossFunction(pub)
   elif cmd == 'search':
       searchFunction(eePub)
   else:
       rospy.logwarn(f"Unknown command: {cmd}")

def tossFunction(pub):
   if not predefinedMatrices:
       rospy.logwarn("No predefined matrices available")
       return
       
   if predefinedMatrices[4] is None:
       rospy.logwarn("No target position in slot 3")
       return

   try:
       target_matrix = predefinedMatrices[4]
       target_matrix[2,3] = -0.005

       pre_toss_matrix = np.copy(target_matrix)

       pre_toss_matrix[0,3] -= 0.1 
       pre_toss_matrix[2,3] += 0.3  # Move up 30cm in Z
       # Move to pre-toss position
       t_pre = matrixToMsg(pre_toss_matrix)

       target_matrix[2,3] += 0.172
       finishing_matrix = np.copy(target_matrix)
       finishing_matrix[0,3] += 0.1
       f_toss = matrixToMsg(finishing_matrix)
       pub.publish(t_pre)
       rospy.sleep(2)
       print("Translated to pre toss position!!!")
       
       # Execute toss motion
       t_toss = matrixToMsg(target_matrix)
       pub.publish(t_toss)
       print("Tossing!!!")
       pub.publish(f_toss)
    
       

       # Clear target position
       predefinedMatrices[4] = None

       
   except Exception as e:
       rospy.logerr(f"Toss execution failed: {e}")

def searchFunction(eePub):
   try:
        eeEnableMsg = Bool(data=False)
        print("Change EE Control STATE")
        eePub.publish(eeEnableMsg)

        rospy.sleep(5)  
        rospy.wait_for_service('search_aruco', timeout=5.0)
        search_client = rospy.ServiceProxy('search_aruco', SearchAruco)
        response = search_client()
       
        if response.success:
            rospy.loginfo(f"Target Searched : Update Target Frame")

            camera_to_target = response.transform
            camera_to_target_np = transformToMatrix(camera_to_target)

            print("Camera to Target : \n,",camera_to_target_np)
    
            world_to_target = np.array( [[1, 0, 0,  predefinedMatrices[2][0,3] -camera_to_target_np[1,3] + 0.073 - 0.175 - 0.045],
                                        [0, -1, 0, predefinedMatrices[2][1,3] -camera_to_target_np[0,3] + 0.18 - 0.329 + 0.00041308 + 0.004],
                                        [0, 0, -1, predefinedMatrices[2][2,3]- 0.25],
                                        [0, 0, 0, 1.0]])
    
            # Store the target location 
            predefinedMatrices[4] = world_to_target
            print("Target : \n,",world_to_target)
    
        eeEnableMsg = Bool(data=True)
        print("Change EE Control STATE")
        eePub.publish(eeEnableMsg)

   except Exception as e:
       rospy.logerr(f"Search transform failed: {e}")
       return None

def main():
    rospy.init_node('rb5_command_node', anonymous=True)
    pub = rospy.Publisher('/rb5_target', TransformStamped, queue_size=10)
    eePub = rospy.Publisher('ee_enable', Bool, queue_size=5)
    sub = rospy.Subscriber('/rb5_keyCommand', String, commandCallback, callback_args=(pub, eePub))

    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
