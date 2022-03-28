# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python
    
import numpy as np
import rospy
import ros_numpy
import tf2_ros

import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String
##TOYOTA SEGMENTATOR
## TOYOTA TABLETOP SEGMENTATOR
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest


"""
import sys
print (sys.path)
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
"""

import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def callback(img_msg):

    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    
    cv2.imshow('xtion rgb  cam'	, cv2_img)
    # Process any keyboard commands
    #keystroke = cv2.waitkey(5)
  
     #print req

    keystroke = cv2.waitKey(1)
    if 32 <= keystroke and keystroke < 128:
        cc = chr(keystroke).lower()
        print cc
        if cc =='r':
    
            res = service_client(req)
            
            rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))
            read_res(res)
            print('Segmentation Requested')
            #print(res)
        
        if cc =='q':
            rospy.signal_shutdown("User hit q key to quit.")


def read_res(res):
	service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
	service_client.wait_for_service(timeout=6.0)
	req = TabletopSegmentationRequest()

	req.crop_x_max = 1.0     # X coordinate maximum value in the area [m]
	req.crop_x_min = -1.0    # X coordinate minimum value in the area [m]
	req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
	req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
	req.crop_z_max = 1.8     # Z coordinate maximum value in the area [m]
	req.crop_z_min = 0.8     # Z coordinate minimum value in the area [m]
	req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
	req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
	req.remove_bg = False    # remove the background of the segment image

	res = service_client(req)
	objs_depth_centroids=[]
	#rospy.loginfo('Number of detected objects={0}'.format(len(   objs_depth_centroids)))
	rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))
	for i in range (len(res.segmented_objects_array.table_objects_array )):
	#print ( 'Plane',i,'has', len(res.segmented_objects_array.table_objects_array[i].depth_image_array), 'objects')
		for j in range (len(res.segmented_objects_array.table_objects_array[i].points_array)):
			pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[j])
			points=np.zeros((pc.shape[0],3))
			points[:,0]=pc['x']
			points[:,1]=pc['y']
			points[:,2]=pc['z']
			objs_depth_centroids.append(np.mean(points,axis=0))

#cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[j] )
#cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[j] )


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a uniques
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global req,service_client , res
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
    service_client.wait_for_service(timeout=6.0)
    req = TabletopSegmentationRequest()

    req.crop_x_max = 1.0     # X coordinate maximum value in the area [m]
    req.crop_x_min = -1.0    # X coordinate minimum value in the area [m]
    req.crop_y_max = 1.0     # Y coordinate maximum value in the area [m]
    req.crop_y_min = -1.0    # Y coordinate minimum value in the area [m]
    req.crop_z_max = 1.8     # Z coordinate maximum value in the area [m]
    req.crop_z_min = 0.8     # Z coordinate minimum value in the area [m]
    req.cluster_z_max = 1.0  # maximum height value of cluster on table [m]
    req.cluster_z_min = 0.0  # minimum height value of cluster on table [m]
    req.remove_bg = False    # remove the background of the segment image

  


    

    
    #rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))

        
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

