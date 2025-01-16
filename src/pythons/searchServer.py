#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform
from sensor_msgs.msg import Image
from rb5_ros_wrapper.srv import SearchAruco, SearchArucoResponse

class ArucoDetectionServer:
   def __init__(self):
       self.bridge = CvBridge()
       self.camera_matrix = np.array([
           [915.289531, 0.000000, 649.404128],
           [0.000000, 918.977925, 354.090401],
           [0.000000, 0.000000, 1.000000]
       ], dtype=np.float64)
       self.dist_coeffs = np.array([0.165794, -0.309514, -0.001978, 0.004553, 0.000000], dtype=np.float64).reshape(5, 1)
       self.search_service = rospy.Service('search_aruco', SearchAruco, self.handle_search_request)

   def handle_search_request(self, req):
       response = SearchArucoResponse()
       try:
           msg = rospy.wait_for_message('/camera/color/image_raw', Image, timeout=5.0)
           image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
           gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
           
           aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
           parameters = aruco.DetectorParameters_create()
           corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                
           if ids is not None:
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners[0], 0.05, self.camera_matrix, self.dist_coeffs)
                rmat = cv2.Rodrigues(rvec)[0]
                r = R.from_matrix(rmat)
                quat = r.as_quat()

                # While Rotation is not necessary 
                # Just Use it as Dummy
                transform = Transform()
                transform.translation.x = tvec[0][0][0]
                transform.translation.y = tvec[0][0][1]
                transform.translation.z = tvec[0][0][2]
                transform.rotation.x = quat[0]
                transform.rotation.y = quat[1]
                transform.rotation.z = quat[2]
                transform.rotation.w = quat[3]
             
                response.success = True
                response.transform = transform
           else:
                response.success = False

       except Exception as e:
           rospy.logerr(f"Error processing image: {e}")
           response.success = False

       return response

if __name__ == '__main__':
   rospy.init_node('aruco_detection_server')
   server = ArucoDetectionServer()
   rospy.spin()