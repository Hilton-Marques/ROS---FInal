#!/usr/bin/env python  
from sys import flags
import rospy

from math import atan2, hypot
import tf2_ros
from geometry_msgs.msg import Twist,Pose2D
from tf2_geometry_msgs import PointStamped

goal = None



def GetGoal(msg):
    #goal_pose = rospy.wait_for_message("/goal", Pose2D)
    global goal
    if (goal is None):
        goal = PointStamped()
    goal.header.stamp = rospy.Time()
    goal.header.frame_id = "odom" # turtle1
    goal.point.x = msg.x
    goal.point.y = msg.y
    goal.point.z = 0.0    #return goal_stamped
    
    #goal = tfBuffer.transform(goal, 'world')
    #goal.header.stamp = rospy.Time.now()

if __name__ == '__main__':
    rospy.init_node('move2goal')
    rospy.Subscriber('/gpg_goal',Pose2D, GetGoal)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(75.0)
    pub = rospy.Publisher('/gpg/mobile_base_controller/cmd_vel' ,Twist, queue_size=100)  
    while (not rospy.is_shutdown()):
        if (goal is None):
            rate.sleep()
            continue
        try:
            #rospy.logwarn(goal)
            trans = tfBuffer.transform(goal, 'gpg')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        rate.sleep()
        sent_msg = Twist()
        norm = hypot(trans.point.x, trans.point.y)
        angle = atan2(trans.point.y, trans.point.x)   
        sent_msg.linear.x = 0.4*0
        sent_msg.linear.y = 0
        sent_msg.linear.z = 0

        sent_msg.angular.z = 0.7*0
        sent_msg.angular.x = 0
        sent_msg.angular.y = 0

        pub.publish(sent_msg)
        #rospy.logwarn('goal_robot')
        #rospy.logwarn(trans)
        #rospy.logwarn('erro')
        #rospy.logwarn(norm)
        if (norm < 0.01):
            goal = None
            #rospy.sleep(1)
            #rospy.logwarn('Get new goal')
        

