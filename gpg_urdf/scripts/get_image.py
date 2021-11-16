#!/usr/bin/env python
from pickle import FALSE, TRUE
import rospy
import cv2 as cv
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
from std_msgs.msg import Float64
import io
import socket
import struct
import numpy as np
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import Vector3Stamped
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
import geometry_msgs.msg

from math import hypot


import tf2_ros

bridge = CvBridge()
cam_model = PinholeCameraModel()
#tfBuffer = tf2_ros.Buffer()
#listener = tf2_ros.TransformListener(tfBuffer)
image = None
image_HSV = None
image_header = None
robot_pose = None
goal_pose = None

def save_image():
    with open('/home/hilton/catkin_ws/src/gpg_urdf/scripts/save2.txt', 'w') as outfile:
        outfile.write('# Array shape: {0}\n'.format(image.shape))
        for data_slice in image:
            np.savetxt(outfile, data_slice, fmt='%-7.2f')
            outfile.write('# New slice\n')
def handle_image(msg):
    global image, image_HSV, image_header
    try:
        image = bridge.imgmsg_to_cv2(msg,"bgr8")
        image_HSV = cv.cvtColor(image,cv.COLOR_BGR2HSV)
        image_header = msg.header
    except CvBridgeError:
        print('error')

def init_camera(data):
    cam_model.fromCameraInfo(data)

def handle_gpg_pose(msg):
    global robot_pose, goal_pose

    robot_pose = Pose2D()
    robot_pose.x =  msg.pose.position.x
    robot_pose.y = msg.pose.position.y
    if goal_pose is not None:
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "sphere"
    
        t.transform.translation.x = goal_pose.x
        t.transform.translation.y = goal_pose.y
        t.transform.translation.z = 0
    
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w
    
        br.sendTransform(t)
    




    
if __name__ == '__main__':
    br = tf2_ros.TransformBroadcaster()
    rospy.init_node('get_image')
    rospy.Subscriber('/gpg/image', Image, handle_image)
    rospy.Subscriber('/gpg/camera_info', CameraInfo , init_camera)
    rospy.Subscriber('/robot_pose',PoseStamped,handle_gpg_pose)
    rate = rospy.Rate(20)          
    pub = rospy.Publisher('/gpg/servo_controller/command' ,Float64, queue_size=100)   
    pub_goal = rospy.Publisher('/gpg_goal',Pose2D, queue_size=10)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        rate.sleep()
        if image is not None:
            #cv.imshow('image_rgb',image)
            #save_image()
            #status = cv.imwrite('/home/hilton/catkin_ws/src/gpg_urdf/scripts/image_saved.jpeg',image)
            #status = np.savetxt('/home/hilton/catkin_ws/src/gpg_urdf/scripts/save.txt', np.array(image), delimiter=',')
            # define range of blue color in HSV
            red = np.uint8([[[0,0,255 ]]])
            hsv_red = cv.cvtColor(red,cv.COLOR_BGR2HSV)
            H = hsv_red[0,0,0]
            lower_blue = np.array([H,50,50])
            upper_blue = np.array([H+5,255,255])
            mask = cv.inRange(image_HSV, lower_blue, upper_blue)
            #kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (5,5))
            #mask = cv.dilate(cv.erode(mask, kernel, iterations=3),kernel, iterations=3)
            # Bitwise-AND mask and original image
            res = cv.bitwise_and(image,image, mask= mask)

            #segmentation
            out = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            contours = out[1]
            max_len = 0
            contour = []
            for i in range(len(contours)):
                c_i = [contours[i]]
                c_j = c_i[0]
                if len(c_j) > max_len:
                    contour = c_i
                    max_len = len(c_j)

            cv.drawContours(image, contour, -1, (0,255,0), 2)
    
            #centroid
            cx = None
            cy = None
            rate.sleep()
            if contour:
                M = cv.moments(contour[0])
                if M['m00'] != 0:
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])
                    cv.circle(image, (cx, cy), 3, (255, 0, 0), 2)
            #rotate servo
            cv.imshow('frame',image)
            cv.waitKey(1) 

            if cx is not None:    
                erro = 160 - cx
                rospy.logwarn(('erro_pixel',erro))
                pub.publish(0.5*erro)
                if abs(erro) > 6:  
                    continue
         
            #find bottom
            if contour:
                c_array = np.array(contour[0])
                c_array = c_array[:,0,:]
                ids = np.argwhere(c_array[:,1] == np.amax(c_array[:,1]))
                pixel_y = c_array[ids[0],1]
                pixel_x = np.uint8(np.rint(np.average(c_array[ids,0])))
                cv.circle(image, (pixel_x, pixel_y), 3, (255, 255, 0), 2)
                #line
                rospy.logwarn(('pixel',pixel_x,pixel_y))
                line = cam_model.projectPixelTo3dRay((pixel_x,pixel_y))
                #rospy.logwarn(line)
                camera_msg = Vector3Stamped()
                camera_msg.header.stamp = rospy.Time() #image_header.stamp 
                camera_msg.header.frame_id = "camera_link"
                camera_msg.vector.x = line[0]
                camera_msg.vector.y = line[1]
                camera_msg.vector.z = line[2]             
                #origin camera                
                camera_msg_o = PointStamped()
                camera_msg_o.header.stamp = rospy.Time() #image_header.stamp #
                camera_msg_o.header.frame_id = "camera_link"
                camera_msg_o.point.x = 0
                camera_msg_o.point.y = 0
                camera_msg_o.point.z = 0
                try:
                    vec_camera = tfBuffer.transform(camera_msg, 'odom')
                    o_camera = tfBuffer.transform(camera_msg_o, 'odom')
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue
                rospy.logwarn(o_camera)
                rospy.logwarn(vec_camera)
                # object position 
                goal_pose = Pose2D()
                lam = - o_camera.point.z / vec_camera.vector.z
                goal_pose.x  = o_camera.point.x + lam*vec_camera.vector.x
                goal_pose.y  = o_camera.point.y + lam*vec_camera.vector.y
                rospy.logwarn(goal_pose)

                #send goal
                if robot_pose is not None:
                    norm = hypot(goal_pose.x - robot_pose.x, goal_pose.y - robot_pose.y)
                    rospy.logwarn(('erro',norm))
                    if norm > 0.5:
                        pub_goal.publish(goal_pose)







            
            cv.imshow('frame',image)
            
           
            #cv.imshow('mask',mask)
            #cv.imshow('res',res)
            
            cv.waitKey(1) 

