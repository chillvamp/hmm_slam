# -*- coding: utf-8 -*-
"""
Created on Sun Jan 19 15:17:33 2020

@author: oscar
"""

#!/usr/bin/env python3
    
import numpy as np
import rospy
import message_filters
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String



import sys
print (sys.path  )
sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')

print (sys.path)
import cv2
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

def callback(img_msg):
    print( "got image",type(img_msg))

    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    print (cv2_img.shape)
    #    cv2.imshow('hand_camera',	cv2_img)

    
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/hsrb/hand_camera/image_raw", Image, callback)
    #rospy.Subscriber("/hsrb/base_scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
