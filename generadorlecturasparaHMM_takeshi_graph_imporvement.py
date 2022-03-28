#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""
from geometry_msgs.msg import Quaternion
import numpy as np
import rospy
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from joblib import dump, load
ccvk = np.load('ccvk.npy')
ccxyth= np.load('ccxyth.npy')



def callback(laser,odometria):
    
        #######################################################
        #centroides = np.load('centroidesVk.npy')
        clf=load('aff_prop_class.joblib_2')

        lec=np.asarray(laser.ranges)
        #print (lec.shape)
        lec[np.isinf(lec)]=13.5
        lec=np.clip(lec,0,5)
        Vk_aff= (int)( clf.predict(lec.reshape(1,-1)))
        #print (range (1,len(lec)))
         
        symbol= np.power(lec.T-ccvk,2).sum(axis=1,keepdims=True).argmin()
        Vk=symbol
        
        xy  = np.asarray((odometria.pose.pose.position.x,odometria.pose.pose.position.y))
        quaternion = (
        odometria.pose.pose.orientation.x,
        odometria.pose.pose.orientation.y,
        odometria.pose.pose.orientation.z,
        odometria.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        
        xyth= np.asarray((odometria.pose.pose.position.x,odometria.pose.pose.position.y,euler[2]))
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        print(str(Vk_aff)+','+str(Vk)+','+str(xythcuant)+ '\n')

        with  open('dataset_hokuyo_wrs/lecs_odom_improvement.txt' , 'a') as out:
            out.write (str(Vk_aff)+','+str(Vk)+','+str(xythcuant)+',' +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )
            #out.write (lec_str +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +","+  str(symbol)+','+str(symbolxy)+'\n' )
        
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    odom  = message_filters.Subscriber('/hsrb/odom',Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,odom],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
