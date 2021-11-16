#!/usr/bin/env python  
import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped


def handle_gpg_pose(msg):
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "gpg"

    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = msg.pose.position.z

    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('gpg_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/robot_pose',
                     PoseStamped,handle_gpg_pose)
    rate = rospy.Rate(50)               
    while not rospy.is_shutdown():
        rate.sleep()
