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
import cv2
from std_msgs.msg import String , ColorRGBA
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker , MarkerArray
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from utils_hmm import  viterbi
from utils_hmm import forw_alg
from utils_hmm import backw_alg
from joblib import dump, load



odom_adjust,odom_adjust_aff=np.zeros(3),np.zeros(3)
first_adjust= np.zeros(3)

first=True
last_states=[]
last_states_2=[]

o_k=[]
o_k2=[]
transitions= np.load('trans.npy')
buf_vit=30
clf=load('aff_prop_class.joblib_2')
marker=Marker()   
markerarray=MarkerArray()




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



    

def callback(laser,pose,odom):
        global xyth , xyth_odom
        global first
        global odom_adjust,odom_adjust_aff,first_adjust , first_adjust_2

        #Print TOPO MAP


        line_color = ColorRGBA()       # a nice color for my line (royalblue)
        line_color.r = 0.254902
        line_color.g = 0.411765
        line_color.b = 0.882353
        line_color.a = 1.0
        start_point = Point()        #start point
        start_point.x = 0.2
        start_point.y = 0.0
        start_point.z = 0.2
        end_point = Point()        #end point
        end_point.x = 0.7
        end_point.y = 0
        end_point.z = 0.2

        marker3 = Marker()
        marker3.id = 3
        marker3.header.frame_id = 'world'
        marker3.type = Marker.LINE_STRIP
        marker3.ns = 'Testline'
        marker3.action = 0
        marker3.scale.x = 0.1
        marker3.points.append(start_point)
        marker3.points.append(end_point)
        marker3.colors.append(line_color)
        marker3.colors.append(line_color)
        pub2.publish(marker3)

            
       
        ##############
        #GET REAL ROBOT POSE 
        text_to_rviz=""
        x_odom=odom.pose.pose.position.x
        y_odom=odom.pose.pose.position.y
        quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        th_odom=euler[2]
    
       
        centroides = np.load('ccvk.npy')
        ccxyth= np.load('ccxyth.npy')
        
        
        
        
        lec=np.asarray(laser.ranges)
        
        
        #GET BOTH O_K 's
        #print (lec.shape)
        lec[np.isinf(lec)]=13.5
        lec=np.clip(lec,0,5)
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        lec.reshape(len(laser.ranges),1 )
        lec=np.clip(lec,0,5    )
        
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
        if len(o_k) >=buf_vit:
            o_k.pop(0)
        o_k.append(symbol)
        if len(o_k2) >=buf_vit:
            o_k2.pop(0)
        o_k2.append(symbol2)
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
        xyth_odom=np.asarray((x_odom, y_odom,th_odom))
        xyth_odom_hmm1=np.asarray((x_odom, y_odom,th_odom))
        xyth_odom_hmm2=np.asarray((x_odom, y_odom,th_odom))
        #QUANTIZING READS (CLASIFIYNG ok2)
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        xyth_odomcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))

        

        if (len(o_k)< buf_vit):
            
            print ( "FILLING BUFFER HMM1")
        
        if (len(o_k)>= buf_vit):
        
            
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            last_states.append((int)(vit_est[-1]))
            
            if (len (last_states)<10 and first)  :
                if (last_states[-1]==xyth_odomcuant):
                    print ("READJUST ODOM, first adjust HMM1",xyth_odom,"to",ccxyth[last_states[-1]]  )
                    odom_adjust= xyth_odom-ccxyth[last_states[-1]]
                    first_adjust=odom_adjust
                    first= False
                else:
                    odom_adjust=first_adjust

               

                
            if len (last_states)>=10:
                last_states.pop(0)
                

                if(last_states[-1]!=last_states[-2]) and (last_states[-1]==xyth_odomcuant):
                    print('TRANSITION DETECTED Hmm1')
                    print('last states',last_states)
                    print ("TRANSITION FROM",last_states[-2],"to",last_states[-1]  )
                    odom_adjust= xyth_odom - transitions[last_states[-2],last_states[-1]]
                    first_adjust=odom_adjust
                    
                    #correct_odom=ccxyth[last_states[-1]]
                    print ("READJUST ODOM",xyth_odom,"to",transitions[last_states[-2],last_states[-1]]  )
                    print(odom_adjust)
                else :
                    odom_adjust=first_adjust
            
                
            
            
            """o_1,o_0=vit_est[-2],vit_est[-1]
            if o_1 != o_0:
                print("HEY",o_0,o_1)
                with  open('dataset_candidatura_wr/transitions.txt' , 'a') as out:
                    out.write (str(o_1)+str(o_0) +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )"""
        
            alpha= forw_alg(o_k,Modelo1)
            print ('Most likely state seq given O',vit_est[-5:])
            #print ('P State given O, M',alpha[:,-1])
            
            vit_est_2= viterbi(o_k2,Modelo2,Modelo2.PI)
            """o_1,o_0=vit_est[-2],vit_est[-1]
            if o_1 != o_0:
                print("HEY",o_0,o_1)
                with  open('dataset_candidatura_wr/transitions_aff.txt' , 'a') as out:
                    out.write (str(o_1)+str(o_0) +str(xyth[0])+","+ str(xyth[1])  +","+str(euler[2])  +'\n' )"""
            alpha= forw_alg(o_k2,Modelo2)
            #print ('P State given O, M_aff_prop',alpha[:,-1])
            print ('Most likely states given O Modelo aff prop (',vit_est_2[-5:])
        
        print('lec vk_aff'+str(Vk_aff)+'lec vk'+str(symbol)  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        print('Wheel (unadjusted) ODOM' ,xyth_odom)
        print('Wheel (adjusted) ODOM HMM1' ,xyth_odom-odom_adjust)
        print ('Pose',xyth)
        text_to_rviz+= 'REAL Pose'+ str(xyth)+'\n'+'Wheel odom error '+ str(np.linalg.norm(xyth[:2]-xyth_odom[:2]))+'\n'+'HMM1 error '+str(np.linalg.norm(xyth[:2]-(xyth_odom[:2]-odom_adjust.T[:2] )))+'\n'
        #text_to_rviz+= 'REAL Pose'+ str(xyth)+'\n'+'Wheel odom '+ str(xyth-xyth_odom)+'\n'+'HMM1 '+str(xyth-(xyth_odom-odom_adjust))+'\n'

        #Getting CORRECTION FROM HMM2

        if (len(o_k2)< buf_vit):
           
            first_adjust_2=odom_adjust_aff
            print("Filling Buffer HMM2")
        
        if (len(o_k2)>= buf_vit):
        
            
            last_states_2.append((int)(vit_est_2[-1]))
            if (len (last_states_2)<10) and (last_states_2[-1]==xyth_odomcuant):
                global first_adjust_2
                print ("READJUST ODOM FIRST ADJUST HMM2",xyth_odom,"to",ccxyth[last_states_2[-1]]  )
                odom_adjust_aff= xyth_odom-ccxyth[last_states_2[-1]]
                first_adjust_2=odom_adjust_aff
                print (odom_adjust_aff)
                
            if len (last_states_2)>=10:
                
                print ("Last states", last_states_2[-1],last_states[-1])
                last_states_2.pop(0)
                if(last_states_2[-1]!=last_states_2[-2])  and (last_states_2[-1]==xyth_odomcuant):
                    print('Trans AFF detect4ed')
                    print('last states_2',last_states_2)
                    print ("TRANSITION FROM",last_states_2[-2],"to",last_states_2[-1]  )
                    odom_adjust_aff= xyth_odom - transitions[last_states_2[-2],last_states_2[-1]]
                    first_adjust_2=odom_adjust_aff
                    
                    #correct_odom=ccxyth[last_states_2[-1]]
                    print ("READJUST ODOM HMM2",xyth_odom,"to",transitions[last_states_2[-2],last_states_2[-1]]  )
                    print(odom_adjust_aff)
                else :
                    odom_adjust_aff=first_adjust_2


            print('Wheel (adjusted) ODOM HMM2 ' ,xyth_odom-odom_adjust_aff)
        text_to_rviz+= 'HMM2 error '+str(np.linalg.norm(xyth[:2]-(xyth_odom[:2]-odom_adjust_aff.T[:2])))+'\n'  
        #text_to_rviz+= 'HMM2 '+str((xyth_odom-odom_adjust_aff))+'\n'
        text_line=""
        for value in xyth_odom:
            text_line+=str(value)+','
        for value in xyth_odom-odom_adjust:
            text_line+=str(value)+','
        for value in xyth_odom-odom_adjust_aff:
            text_line+=str(value)+','
        for value in xyth:
            text_line+=str(value)+','        
        text_line+='\n'


        print (text_line)



        marker.header.frame_id="/map"
        marker.header.stamp = rospy.Time.now()
        marker.id=1

        
        marker.scale.z = 0.2
        marker.pose.position.x = xyth[0]+0.5
        marker.pose.position.y = xyth[1]+0.5  
        marker.pose.position.z = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text=text_to_rviz
        markerarray.markers.append(marker)
        pub.publish(markerarray)

        with  open('dataset_candidatura_wr/odometry_and_odometrycorrected.txt' , 'a') as out:
            text_line=""
            for value in xyth_odom:
                text_line+=str(value)+','
            for value in xyth_odom-odom_adjust:
                text_line+=str(value)+','
            for value in xyth_odom-odom_adjust_aff:
                text_line+=str(value)+','
            for value in xyth:
                text_line+=str(value)+','        
            text_line+='\n'
            out.write(text_line)

            """    with  open('dataset_candidatura_wr/estimadores.txt' , 'a') as out:
                            text_line=""
                            for value in xyth_odom:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust_aff:
                                text_line+=str(value)+','
                            for value in xyth:
                                text_line+=str(value)+','        
                            text_line+= str(last_states[-1])+','+str(last_states[-1])+','
                            text_line+='\n'
                            out.write(text_line)
                    """    
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #clf=load('aff_prop_class.joblib')
    #global pub
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    pub= rospy.Publisher('aa/Viterbi',MarkerArray,queue_size=1)
    pub2 = rospy.Publisher('/aa/HMM_topo/', MarkerArray, queue_size=1)  
    #pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    odom= message_filters.Subscriber("/hsrb/wheel_odom",Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose,odom],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    
    
   
    listener()
