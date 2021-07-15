#!/home/loris/anaconda3/envs/py3.7/bin/python

# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 10:02:22 2020

@author: marco
"""
import sys
import pyrealsense2 as rs
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt 
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import rospy
import sensor_msgs.point_cloud2 as pcl2
import open3d as o3d
from planner_online.msg import start_sensor
from planner_online.msg import stop_sensor
import re
global msg
from std_msgs.msg import String
global stopnode
#import json
# import tf2_ros
# import tf2_geometry_msgs #import the packages first

#CHANGE ROOT

param_file_name = "//home/loris/ply_and_stl/param.txt"
min_z = .3
max_z = 0.8
x_resolution = 1280
y_resolution = 720
fps = 30

def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring


def empty(a):
    pass

def callback(stop_sensor):
    if stop_sensor.stop ==1:
        global msg
        msg=1
def filtra_e_salva(depth_frame,min_dist,max_dist,filename):

    # pc.map_to(color)
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices())
    x = vtx['f0']
    y = vtx['f1']
    z = vtx['f2']
    my_pointcloud = np.vstack((x,y,z))
    pcdx = np.transpose(my_pointcloud)
    now=1
    if now==1:# or curr_frame==39:
        lista=[]
        for aa in range(0, len(pcdx)):
            if pcdx[aa,2]<max_dist and pcdx[aa,2]>min_dist:
                lista.append(aa)
                
    point_cloud3 = pcdx[lista,:]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud3)    
    
    # pcd1 = pcd.crop(pcd,)
    
    o3d.io.write_point_cloud(filename,pcd) 
    
    
    
    # Cleanup:
    # pipe.stop()
    print("Frames Captured")

file = open(param_file_name,"r")
parameters = file.readlines()
scan_name = taglia_stringa("{(.*?)}",parameters[48],"testo") 
scan_dir = taglia_stringa("{(.*?)}",parameters[47],"testo")
scan_raw= scan_dir+"scan_raw.ply"

rospy.init_node('pub_POINTCLOUD')
pcl_pub = rospy.Publisher("/pointcloud_visualizer", PointCloud2, queue_size =10)
ref_sys = 'panda_hand'
ref_sys = 'Camera'

print("POINTCLOUD NODE\n")




stopnode =  '0'
print("POINT RECORDING RANGE: %1.2f" %min_z ,"- %1.2f m"  %max_z)
# Start streaming
while not rospy.is_shutdown():
    sensor_start = start_sensor()
    

    end = rospy.wait_for_message('/close_nodes', String)
    if end.data == 'CLOSE':
        sys.exit()
    print("WAITING FOR THE MESSAGE TO START THE SENSOR")
    sensor_start = rospy.wait_for_message('sensor_start',start_sensor)
 
    if sensor_start.start==0:
        
        print("MESSAGE RECEIVED: Publishing PointCloud")

        curr_frame = 0
        try:
            while True and rospy.is_shutdown()==False:
                msg = 0

                pc=rs.pointcloud()
                pipe = rs.pipeline()
                cfg = rs.config()
                points = rs.points()
                cfg.enable_stream(rs.stream.depth,x_resolution,y_resolution, rs.format.z16,fps)
                profile = pipe.start(cfg)
                device = profile.get_device()
                depth_sensor = device.query_sensors()[0]
                laser_pwr = depth_sensor.get_option(rs.option.laser_power)
                set_laser = 360
                depth_sensor.set_option(rs.option.laser_power, set_laser)
                depth_sensor.set_option(rs.option.hdr_enabled, 0.0)
                colorizer = rs.colorizer()



                depth_to_disparity = rs.disparity_transform(True)
                disparity_to_depth = rs.disparity_transform(False)
                print("laserpower:",laser_pwr)

                frames = []
                for x in range(15):
                    frameset = pipe.wait_for_frames()
                    frames.append(frameset.get_depth_frame())
                frames = []
                for x in range(15):
                    frameset = pipe.wait_for_frames()
                    frames.append(frameset.get_depth_frame())

                pipe.stop()
                print("Frames Captured")
                spatial = rs.spatial_filter()
                spatial.set_option(rs.option.filter_magnitude, 1)
                spatial.set_option(rs.option.filter_smooth_alpha, 0.25)
                spatial.set_option(rs.option.filter_smooth_delta, 1)
                temporal = rs.temporal_filter(smooth_alpha=0,smooth_delta=100,persistence_control=3)
                # temporal.set_option(rs.option.filter_smooth_alpha, .4)
                # temporal.set_option(rs.option.filter_smooth_delta, 20)
                # temporal.set_option(rs.option.persistence_control, 8)
                for x in range(15):
                    frame = frames[x]
                    #frame = depth_to_disparity.process(frame)
                    frame = spatial.process(frame)
                    frame = temporal.process(frame)
                    #frame = disparity_to_depth.process(frame)


                filtra_e_salva(frame,min_z,max_z,scan_name)
                filtra_e_salva(frames[14],min_z,max_z,scan_raw)
                curr_frame = 1

                
                rospy.Subscriber('sensor_stop', stop_sensor, callback)
                
                if curr_frame == 1 or msg==1:
                    print("SENSOR STOPPED")
                    print('--------------------------------------------------------------\n')
                    break

            
        finally:

                cv2.destroyAllWindows()
                
