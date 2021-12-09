#!/usr/bin/env python
from pickle import FALSE, TRUE
import rospy
import cv2 as cv
from rospy.timer import sleep
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from cv_bridge import CvBridge , CvBridgeError
from std_msgs.msg import Float64
import sys
import io
import socket
import struct
import numpy as np
from image_geometry import PinholeCameraModel
from tf2_geometry_msgs import Vector3Stamped
from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import PoseStamped
from gpg_urdf.msg import goal_list
import geometry_msgs.msg

from math import hypot


import tf2_ros

bridge = CvBridge()
cam_model = PinholeCameraModel()
#tfBuffer = tf2_ros.Buffer()
#listener = tf2_ros.TransformListener(tfBuffer)
image = None
image_header = None
robot_pose = None
FLOAT_MAX = sys.float_info.max
goal_pose = Pose2D()
goal_pose.x = FLOAT_MAX
goal_pose.y = FLOAT_MAX
goals_list = None
total_goals = None
count_goals = 0
is_object_found = False
contour = None
norm_goal = 0
pub_goal = None
class yolov3:
   def __init__(self):
       self.classes = open('/home/hilton/catkin_ws/src/gpg_urdf/scripts/coco.names').read().strip().split('\n')
       # Give the configuration and weight files for the model and load the network.
       self.net = cv.dnn.readNetFromDarknet('/home/hilton/catkin_ws/src/gpg_urdf/scripts/yolov3.cfg', '/home/hilton/catkin_ws/src/gpg_urdf/scripts/yolov3.weights')
       self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
       ln = self.net.getLayerNames()
       self.ln = [ln[i - 1] for i in self.net.getUnconnectedOutLayers()]
       self.boxes = []
       self.confidences = []
       self.classIDs = []
       self.names = []
   def reset(self):
       self.boxes = []
       self.confidences = []
       self.classIDs = []
       self.names = []
   def predict(self,img):
       blob = cv.dnn.blobFromImage(img, 1/255.0, (416, 416), swapRB=True, crop=False)
       self.net.setInput(blob)
       outputs = self.net.forward(self.ln)
       h, w = img.shape[:2]
       self.reset()
       for output in outputs:
           for detection in output:
               scores = detection[5:]
               classID = np.argmax(scores)
               confidence = scores[classID]
               if confidence > 0.5:
                   box = detection[:4] * np.array([w, h, w, h])
                   (centerX, centerY, width, height) = box.astype("int")
                   x = int(centerX - (width / 2))
                   y = int(centerY - (height / 2))
                   box = [x, y, int(width), int(height)]
                   self.boxes.append(box)
                   self.confidences.append(float(confidence))
                   self.classIDs.append(classID)
                   self.names.append(self.classes[classID])            
       indices = cv.dnn.NMSBoxes(self.boxes, self.confidences, 0.5, 0.4)
       boxes_out = []
       confidences_out = []
       names_out = []
       if len(indices) > 0:
           for i in indices.flatten():
               boxes_out.append(self.boxes[i])
               confidences_out.append(self.confidences[i])
               names_out.append(self.classes[self.classIDs[i]])
           return (boxes_out,confidences_out,names_out)
       else:
           return []
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
        #image_header = msg.header
        image_header = rospy.Time()
        #image_header = rospy.Time.now()
    except CvBridgeError:
        print('error')

def get_goals_list(msg):
    global goals_list, total_goals
    goals_list = []
    total_goals = len(msg.goals)
    for i in range(total_goals):
        goals_list.append(msg.goals[i])
def init_camera(data):
    cam_model.fromCameraInfo(data)

def handle_gpg_pose(msg):
    global robot_pose, goal_pose, count_goals,is_object_found,flag_goal,count_shoots
    robot_pose = Pose2D()
    robot_pose.x =  msg.pose.position.x
    robot_pose.y = msg.pose.position.y
    if goal_pose.x != FLOAT_MAX:
        norm = hypot(goal_pose.x - robot_pose.x, goal_pose.y - robot_pose.y)
        if norm < 0.7:
            if is_object_found:
                count_goals = (count_goals + 1) % total_goals
            goal_pose = Pose2D()
            goal_pose.x = FLOAT_MAX
            goal_pose.y = FLOAT_MAX
            is_object_found = False
            rospy.logwarn(('objeto atingido' , count_goals) )




    
if __name__ == '__main__':
    br = tf2_ros.TransformBroadcaster()
    rospy.init_node('get_image')
    rospy.Subscriber('/gpg/image', Image, handle_image)
    rospy.Subscriber('/gpg/camera_info', CameraInfo , init_camera)
    rospy.Subscriber('/robot_pose',PoseStamped,handle_gpg_pose)
    rospy.Subscriber('/gpg/goal_list',goal_list, get_goals_list)
    rate = rospy.Rate(20)   
    yolo_obj = yolov3()       
    pub = rospy.Publisher('/gpg/servo_controller/command' ,Float64, queue_size=100)   
    pub_goal = rospy.Publisher('/gpg_goal',Pose2D, queue_size=100)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    while not rospy.is_shutdown():
        rate.sleep()
        if goals_list is not None:
            if image is not None:
                search = image 
                pub_goal.publish(goal_pose)
                contour = yolo_obj.predict(image)
                if contour:
                    if not is_object_found:
                        object = goals_list[count_goals]
                        names = contour[2]
                        for i in range(len(names)):
                            boxe = contour[0][i]
                            precision = contour[1][i]
                            name = contour[2][i]
                            (x, y) = (boxe[0], boxe[1])
                            (w, h) = (boxe[2], boxe[3])
                            cx = int(np.rint(x+w/2))
                            cy = int(np.rint(y+h/2))
                            xA = int(np.rint(x+w))
                            yA = int(np.rint(y+h))
                            text = "{}: {:.4f}".format(name, precision)
                            search = image
                            cv.rectangle(search, (x, y), (x + w, y + h), [255,0,255], 2)
                            cv.putText(search, text, (xA, yA), cv.FONT_HERSHEY_SIMPLEX, 0.5, [0,0,255], 1)
                            cv.circle(search, (cx, cy), 3, (255, 0, 255), 1)
                            cv.imshow('search',search)
                            cv.waitKey(1)                             
                            if object == names[i]:
                                is_object_found = True
                                rospy.logwarn('oi')
                                pub.publish(0.0)
                                break
                        rospy.logwarn('Procurando objeto')
                        pub.publish(0.1)
                        cv.imshow('search',search)
                        cv.waitKey(1)  
                    else:
                        object = goals_list[count_goals]
                        names = contour[2]
                        rospy.logwarn((object, names))
                        flag = False
                        for i in range(len(names)):
                            boxe = contour[0][i]
                            precision = contour[1][i]
                            name = contour[2][i]
                            if name == object:
                                flag = True
                                break
                        if not flag:
                            continue 
                        rospy.logwarn((object,name))
                        (x, y) = (boxe[0], boxe[1])
                        (w, h) = (boxe[2], boxe[3])
                        cx = int(np.rint(x+w/2))
                        cy = int(np.rint(y+h/2))
                        xA = int(np.rint(x+w))
                        yA = int(np.rint(y+h))
                        frame = image
                        cv.rectangle(search, (x, y), (x + w, y + h), [255,0,0], 2)
                        text = "{}: {:.4f}".format(name, precision)
                        pixel_x = int(np.rint(x+w/2))
                        pixel_y = int(y+h)
                        #rospy.logwarn(('cx',cx,cy))
                        cv.circle(search, (pixel_x, pixel_y), 3, (255, 0, 0), 1)
                        cv.putText(search, text, (xA, yA), cv.FONT_HERSHEY_SIMPLEX, 0.5, [0,0,255], 1)
                        cv.circle(search, (cx, cy), 3, (255, 0, 0), 1)
                        cv.imshow('search',search)
                        cv.waitKey(2) 
                        #fix the camera
                        erro = 160 - cx
                        rospy.logwarn(('erro_pixel',erro))
                        vel = 0.0008*erro
                        pub.publish(vel)
                        if abs(erro) > 7:  
                            continue
                        #find global pose
                        line = cam_model.projectPixelTo3dRay((pixel_x ,pixel_y))
                        if line[1] < 0:
                            continue
                        camera_msg = Vector3Stamped()
                        camera_msg.header.stamp = image_header
                        camera_msg.header.frame_id = "camera_link"
                        camera_msg.vector.x = line[0]
                        camera_msg.vector.y = line[1]
                        camera_msg.vector.z = line[2]             
                    #origin camera                
                        camera_msg_o = PointStamped()
                        camera_msg_o.header.stamp = image_header #
                        camera_msg_o.header.frame_id = "camera_link"
                        camera_msg_o.point.x = 0
                        camera_msg_o.point.y = 0
                        camera_msg_o.point.z = 0
                        try:
                            vec_camera = tfBuffer.transform(camera_msg, 'odom')
                            o_camera = tfBuffer.transform(camera_msg_o, 'odom')
                        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                            rospy.logwarn('hey')
                            continue
                        #rospy.logwarn(o_camera)
                        #rospy.logwarn(vec_camera)
                        # object position 
                        goal = Pose2D()
                        #seems more accurate
                        if robot_pose is not None:
                            o_camera.point.x = robot_pose.x + 0.075
                            o_camera.point.y = robot_pose.y + 0.0
                            o_camera.point.z = 0.065
                        lam = - o_camera.point.z / vec_camera.vector.z
                        goal.x  = o_camera.point.x + lam*vec_camera.vector.x
                        goal.y  = o_camera.point.y + lam*vec_camera.vector.y
                        rospy.logwarn(('goal',goal))
                        norm = hypot(goal.x, goal.y)
                        if goal_pose.x == FLOAT_MAX:
                            if norm < 14: #max radius
                                norm_goal = norm
                                goal_pose = goal
                                #rospy.logwarn(('goal_pose',goal_pose))
                                pub_goal.publish(goal_pose)
                        else:
                            if norm < norm_goal:
                                goal_pose = goal
                                rospy.logwarn(('goal_exact',goal_pose))
                                norm_goal = norm
                                pub_goal.publish(goal_pose)
                                
                            
                                
                    #cv.waitKey(1)         
                    
                else:
                    if not is_object_found:
                        cv.imshow('search',search)
                        cv.waitKey(3)
                        rospy.logwarn('procurando -.-')
                        pub.publish(0.15)   
                





