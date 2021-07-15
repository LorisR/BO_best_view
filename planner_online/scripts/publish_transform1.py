#!/home/marco/anaconda3/envs/py27/bin/python
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
        br.sendTransform((-0.5732233 ,  0.0732233 ,  0.58193977), 
                         (0.54489511,  0.81549316, -0.16221167, -0.10838638),
                         rospy.Time.now(),
                         "Camera",
                         "panda_link0")
        
    end = rospy.wait_for_message('/close_nodes', String)
    if end.data == 'CLOSE':
            sys.exit()
    #rate.sleep()