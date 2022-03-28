# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python
    
import numpy as np
import rospy
import ros_numpy

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
    print( "got image",type(img_msg))

    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    print (cv2_img.shape)
    cv2.imshow('xtion rgb  cam'	, cv2_img)
    # Process any keyboard commands
    #keystroke = cv2.waitkey(5)
  
     #print req

    keystroke = cv2.waitKey(1)
    if 32 <= keystroke and keystroke < 128:
    	cc = chr(keystroke).lower()
    	print cc
    	if cc =='r':
    		req=TabletopSegmentationRequest()
    		print (req)
    	if cc =='q':
    		rospy.signal_shutdown("User hit q key to quit.")

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global req
    rospy.init_node('listener', anonymous=True)
    service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
    service_client.wait_for_service(timeout=6.0)
    req = TabletopSegmentationRequest()

    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
        
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()

