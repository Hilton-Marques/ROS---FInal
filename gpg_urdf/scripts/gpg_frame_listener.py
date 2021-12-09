#!/usr/bin/env python  
from sys import flags
import rospy
import sys
from math import atan2, hypot
#from scripts.get_image import FLOAT_MAX
import tf2_ros
from geometry_msgs.msg import Twist,Pose2D
from tf2_geometry_msgs import PointStamped

goal = None

FLT_MAX = sys.float_info.max


def GetGoal(msg):
    global goal
    if (goal is None):
        goal = PointStamped()
    goal.header.stamp = rospy.Time()
    goal.header.frame_id = "odom" 
    goal.point.x = msg.x
    goal.point.y = msg.y
    goal.point.z = 0.0   
    
 

if __name__ == '__main__':
    rospy.init_node('move2goal')
    rospy.Subscriber('/gpg_goal',Pose2D, GetGoal)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(75.0)
    pub = rospy.Publisher('/gpg/mobile_base_controller/cmd_vel' ,Twist, queue_size=100)  
    flag2 = True
    sent_msg = Twist()
    while (not rospy.is_shutdown()):
        if (goal is None):
            rate.sleep()
            continue
        if (goal.point.x == FLT_MAX):
            rate.sleep()
            #desacelerando
            flag2 = True
            sent_msg.linear.x = 0.0
            sent_msg.linear.y = 0
            sent_msg.linear.z = 0

            sent_msg.angular.z = 0.00
            sent_msg.angular.x = 0
            sent_msg.angular.y = 0
            pub.publish(sent_msg)
            continue
        try:
            #rospy.logwarn(goal)
            trans = tfBuffer.transform(goal, 'gpg')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
        norm = hypot(trans.point.x, trans.point.y)
        angle = atan2(trans.point.y, trans.point.x)   

        if abs(angle) > 0.5:
            if flag2:
                sent_msg.linear.x = 0
                sent_msg.linear.y = 0
                sent_msg.linear.z = 0

                sent_msg.angular.z = 0.35*angle
                sent_msg.angular.x = 0
                sent_msg.angular.y = 0
            else:
                sent_msg.linear.x = 3.0*norm
                sent_msg.linear.y = 0
                sent_msg.linear.z = 0

                sent_msg.angular.z = 0.35*angle
                sent_msg.angular.x = 0
                sent_msg.angular.y = 0
        else:        
            flag2 = False
            sent_msg.linear.x = 3.0*norm
            sent_msg.linear.y = 0
            sent_msg.linear.z = 0

            sent_msg.angular.z = 0.35*angle
            sent_msg.angular.x = 0
            sent_msg.angular.y = 0
            
   
        pub.publish(sent_msg)

  