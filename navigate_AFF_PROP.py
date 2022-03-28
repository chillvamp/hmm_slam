n#!/usr/bin/env python2
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
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from utils_hmm import  viterbi
from utils_hmm import forw_alg
from utils_hmm import backw_alg
from joblib import dump, load
global Modelo1
global Modelo2
global o_k
global o_k2
global buf_vit
global clf
o_k=[]
o_k2=[]
buf_vit=100
#clf=load('aff_prop_class.joblib_2')  




class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  

#############
A, B, PI= np.load('A.npy') , np.load('B.npy') , np.load('PI.npy')
Modelo1= HMM(A,B,PI)
A2, B2, PI2= np.load('A2.npy') , np.load('B2.npy') , np.load('PI2.npy')
Modelo2= HMM(A2,B2,PI2)

def readPoint(punto):
     
     global xcl,ycl,path,thetacl
     
     thetacl=0
     point= punto
     
          
     xcl =punto.point.x
     ycl =punto.point.y
     if len(path)>=300:
         point.point.x , point.point.y=   0.978202  ,  1.0
     else:
         point.point.x , point.point.y=  0.301959  ,  1.15 
     
     
     pub.publish(point)

def callback(laser,pose):
        
        
        centroides = np.load('ccvk.npy')
        ccxyth= np.load('ccxyth.npy')
        
        
        
        
        lec=np.asarray(laser.ranges)
        
        
        
        #print (lec.shape)
        lec[np.isinf(lec)]=13.5
        lec=np.clip(lec,0,5)
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )
        
        Vk_aff= (int)( clf.predict(lec.reshape(1,-1)))
        
       
        
#        
        quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmin()
        symbol2= Vk_aff
        path.append(Vk_aff )
        if len(path)>=301:
         print('yey')
         path.pop(0)
     
        
       
      
        """
        if len(o_k) >=buf_vit:
            o_k.pop(0)
        o_k.append(symbol)
        if len(o_k2) >=buf_vit:
            o_k2.pop(0)
        o_k2.append(symbol2)
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
      
        
        if (len(o_k)== buf_vit):
            #print (o_k)
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            alpha= forw_alg(o_k,Modelo1)
            print ('Most likely state seq given O',vit_est[-5:])
           # print ('P State given O, M',alpha[:,-1])
            
            vit_est= viterbi(o_k2,Modelo2,Modelo2.PI)
            alpha= forw_alg(o_k2,Modelo2)
            #print ('P State given O, M_aff_prop',alpha[:,-1])
            print ('Most likely states given O Modelo aff prop (',vit_est[-5:])
            """
        print('lec vk_aff'+str(Vk_aff)+'lec vk'+str(symbol))#+' Pose ('  +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    global clf,pub
    clf=load('aff_prop_class.joblib_2')
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher("/clicked_point",PointStamped, queue_size=1)
    
    
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    global path
    
    path=[]
    
    
   
    listener()
