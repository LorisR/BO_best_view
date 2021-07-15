#!/home/loris/anaconda3/envs/py2/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  7 16:03:09 2020

@author: marco
"""


import roslib
import rospy
import tf
import math
from std_msgs.msg import String
import sys

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    print('Camera TF CREATED')
    while not rospy.is_shutdown():
        br.sendTransform((-0.02,0,0.05285), 
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "Camera",
                         "panda_hand")
        
    end = rospy.wait_for_message('/close_nodes', String)
    if end.data == 'CLOSE':
            sys.exit()
    #rate.sleep()