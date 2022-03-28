    #!/usr/bin/env python
# Moveit chillvamp@hotmail.com
# OSCII CODE
# Copyright (C) 2017 Toyota Motor Corporation
import os
import sys
import geometry_msgs.msg
from sensor_msgs.msg import Image
#import moveit_commander
#import moveit_msgs.msg
import rospy
import tf
import ros_numpy
import numpy as np

## TOYOTA TABLETOP SEGMENTATOR
from tmc_tabletop_segmentator.srv import TabletopSegmentation
from tmc_tabletop_segmentator.srv import TabletopSegmentationRequest


##CV2 
from cv_bridge import CvBridge, CvBridgeError

import cv2

## TF 2's ( not tensorflow2 but TF 2)
import tf2_ros
cv2_img=np.zeros((100,100,3))
bridge = CvBridge()


def read_tf(frame1,frame2):
   
    try:
        #tfBuffer.lookup_transform( "head_rgbd_sensor_gazebo_frame",'map', rospy.Time())
        trans = tfBuffer.lookup_transform( frame1,frame2, rospy.Time(0))
        quat= trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w
        euler= tf.transformations.euler_from_quaternion((trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w))
        print(trans)
        return (trans)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('no tf')
        return (False)


def callback(img_msg):

 
    cv2_img = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    cv2.imshow('xtion rgb  cam' , cv2_img)
    


    keystroke = cv2.waitKey(1)
    if 32 <= keystroke and keystroke < 128:
        cc = chr(keystroke).lower()
        print cc
        if cc =='r':
    
            print('Segmentation Requested')
            croped_images=image_tag()
            for i in range(len(croped_images)):
                print (croped_images[i].shape)
                cv2.imshow('croped'+str(i)    , croped_images[i])
        if cc =='s':
            if (os.path.isdir(   str(os.getcwd())+'/imagesfromlistener'  ) ):

                print('SAving image in ', str(os.getcwd())+'/imagesfromlistener')
                cap_cnt= len(os.listdir ('imagesfromlistener'))+1
                cap_name = "/home/oscar/Codes/roscodes/scripts_ros/imagesfromlistener/image_from_listener_{}.png".format(cap_cnt)
                #cap_name = "imagesfromlistener/image_from_listener_{}.png".format(cap_cnt)
                cv2.imwrite(cap_name, cv2_img)
                print("Saving {}!".format(cap_name))
            else :
                print('create dir ' ,str(os.getcwd())+'/imagesfromlistener')  
    
            
        
        if cc =='q':
            rospy.signal_shutdown("User hit q key to quit.")
        

        
def image_tag():
    
    ### DETECT PLANES

    req.crop_enabled = True
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
    #### READ SERVICE  RESPONSE 
    croped_images=[]
    objs_depth_centroids=[]
    for i in range (len(res.segmented_objects_array.table_objects_array )):
        #print ( 'Plane',i,'has', len(res.segmented_objects_array.table_objects_array[i].depth_image_array), 'objects')
        for j in range (len(res.segmented_objects_array.table_objects_array[i].points_array)):
            #cv2_img_depth = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].depth_image_array[j] )
            cv2_img = bridge.imgmsg_to_cv2(res.segmented_objects_array.table_objects_array[i].rgb_image_array[j] )
            pc= ros_numpy.numpify (res.segmented_objects_array.table_objects_array[i].points_array[j])
            points=np.zeros((pc.shape[0],3))
            points[:,0]=pc['x']
            points[:,1]=pc['y']
            points[:,2]=pc['z']
            objs_depth_centroids.append(np.mean(points,axis=0))
            print(cv2_img.shape)
            croped_images.append(cv2_img)
            
            
            





    rospy.loginfo('Number of detected objects={0}'.format(len(   objs_depth_centroids)))
    rospy.loginfo('Number of detected planes={0}'.format(len(res.table_array.tables)))
    static_transformStamped = geometry_msgs.msg.TransformStamped()
    trans=read_tf('head_rgbd_sensor_gazebo_frame','map')
    if trans == False:
        print('no tf')
    else:
        print(trans)
        quat= trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w
        euler= tf.transformations.euler_from_quaternion((trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w))

        for ind, xyz in enumerate(objs_depth_centroids):
        
            
            #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obj"+str(ind), "head_rgbd_sensor_link")
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "head_rgbd_sensor_link"
            static_transformStamped.child_frame_id = "TF2_"+(str)(ind)
            static_transformStamped.transform.translation.x = float(xyz[0])
            static_transformStamped.transform.translation.y = float(xyz[1])
            static_transformStamped.transform.translation.z = float(xyz[2])
            #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
        
            static_transformStamped.transform.rotation.x = -quat[0]#trans.transform.rotation.x
            static_transformStamped.transform.rotation.y = -quat[1]#trans.transform.rotation.y
            static_transformStamped.transform.rotation.z = -quat[2]#trans.transform.rotation.z
            static_transformStamped.transform.rotation.w = -quat[3]#trans.transform.rotation.w
        
            tf_broadcaster.sendTransform(static_transformStamped)
    return croped_images
    
    
    #cv2.imshow("rgb", cv2_img) 
    #print len(objs_depth_centroids)

    ###PUBLISH TF
    #obj_size = [0.1, 0.1, 1.1]
    #static_transformStamped = geometry_msgs.msg.TransformStamped()
    #try:
    #    trans = tfBuffer.lookup_transform( "head_rgbd_sensor_gazebo_frame",'map', rospy.Time())
    #except:
    #    print ('tf failiure')
    
    


    #for ind, xyz in enumerate(objs_depth_centroids):
    #
    #    
    #    #tf_broadcaster1.sendTransform( (xyz[0],xyz[1],xyz[2]),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(), "obj"+str(ind), "head_rgbd_sensor_link")
    #    static_transformStamped.header.stamp = rospy.Time.now()
    #    static_transformStamped.header.frame_id = "head_rgbd_sensor_link"
    #    static_transformStamped.child_frame_id = "TF2_"+(str)(ind)
    #    static_transformStamped.transform.translation.x = float(xyz[0])
    #    static_transformStamped.transform.translation.y = float(xyz[1])
    #    static_transformStamped.transform.translation.z = float(xyz[2])
    #    #quat = tf.transformations.quaternion_from_euler(-euler[0],0,1.5)
    #
    #    static_transformStamped.transform.rotation.x = -quat[0]#trans.transform.rotation.x
    #    static_transformStamped.transform.rotation.y = -quat[1]#trans.transform.rotation.y
    #    static_transformStamped.transform.rotation.z = -quat[2]#trans.transform.rotation.z
    #    static_transformStamped.transform.rotation.w = -quat[3]#trans.transform.rotation.w
    #
    #    tf_broadcaster.sendTransform(static_transformStamped)
    #
    #
    #
    #    rospy.sleep(.5)
    #    trans = tfBuffer.lookup_transform( 'map',"TF2_"+(str)(ind), rospy.Time())
    #    trans.header.frame_id='map'
    #    trans.child_frame_id = "STATIC"+(str)(ind)
    #    tf_static_broadcaster.sendTransform(trans)

    rospy.sleep(.1)
    rospy.spin()

            

def image_tag_listener():
    
    image_tag()
    
    rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_rect_color", Image, callback)
    #while not rospy.is_shutdown():
    #    try:
    #        #tfBuffer.lookup_transform( "head_rgbd_sensor_gazebo_frame",'map', rospy.Time())
    #        trans = tfBuffer.lookup_transform( 'head_rgbd_sensor_gazebo_frame','map', rospy.Time(0))
    #        print(trans)
    #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #       print('no tf')
    #       continue
    
    rospy.spin()

if __name__ == "__main__":
    global req , service_client , tf_broadcaster1
    rospy.init_node("image_tag", anonymous=True)
    service_client = rospy.ServiceProxy('/tabletop_segmentator_node/execute', TabletopSegmentation)
    service_client.wait_for_service(timeout=6.0)
    rate = rospy.Rate(10.0)
    tf_broadcaster =tf2_ros.TransformBroadcaster() # tf2_ros.StaticTransformBroadcaster()
    tf_static_broadcaster= tf2_ros.StaticTransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    req = TabletopSegmentationRequest()
    print req
    image_tag_listener()
