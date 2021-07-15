#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep  2 17:48:33 2020

@author: marco
"""
from mpl_toolkits import mplot3d
import numpy as np
import math
import matplotlib.pyplot as plt 
import cv2 as cv
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


class sphere_generator():
    

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + '%s', data.data)
        print(data.data)
        global sample_points
        self.sample_points = data.data
        sample_points = self.sample_points
        

    def listener(self):
        rospy.init_node('sphere_coordinates_listener', anonymous=True)
        rospy.Subscriber('sphere_values_normals', Float32MultiArray, self.callback)
        rospy.spin

    
if __name__ == '__main__':
    
   xyz = sphere_generator()
   xyz.listener()
   mici = xyz.sample_points
   
    
    


