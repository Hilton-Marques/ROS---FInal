#! /usr/bin/env python
import rospy
import sys

from rospy.core import is_shutdown
from gpg_urdf.msg import goal_list


def SendGoal():
    rospy.init_node('send_goal',anonymous=True)
    pub = rospy.Publisher('/gpg/goal_list',goal_list, queue_size=10)
    #msg = ('person','truck','tree','aaa')
    rate = rospy.Rate(10)
    while (not rospy.is_shutdown()): 
        #pub.publish(msg)
        rate.sleep()



if __name__ == "__main__":
    try:
        SendGoal()
    except rospy.ROSInterruptException:
        pass
