#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry

def callback(msg):
    new_msg = msg
    new_msg.header.frame_id = rospy.get_param("~frame_id", "odom_new")          # 默认改父坐标系
    # new_msg.child_frame_id  = rospy.get_param("~child_frame_id", "base_link")   # 默认改子坐标系
    pub.publish(new_msg)

if __name__ == "__main__":
    rospy.init_node("odom_frame_changer")

    sub_topic = rospy.get_param("~input_topic", "/odom")
    pub_topic = rospy.get_param("~output_topic", "/odom_fixed")

    rospy.loginfo("Listening on %s, publishing on %s", sub_topic, pub_topic)

    pub = rospy.Publisher(pub_topic, Odometry, queue_size=10)
    sub = rospy.Subscriber(sub_topic, Odometry, callback, queue_size=10)

    rospy.spin()
# rosrun development_tools odom_frame_changer.py _input_topic:=/mavros/local_position/odom _output_topic:=/sim/mavros/local_position/odom _frame_id:=world 


