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
global msg
from std_msgs.msg import String

# import tf2_ros
# import tf2_geometry_msgs #import the packages first


def empty(a):
    pass

def callback(stop_sensor):
    if stop_sensor.stop ==1:
        global msg
        msg=1



rospy.init_node('pub_OCTOMAP')
print("OCTOMAP NODE\n")
pcl_pub = rospy.Publisher("/camera/depth_registered/points", PointCloud2, queue_size =10)
ref_sys = 'panda_hand'
ref_sys = 'Camera'

x_resolution = 640
y_resolution = 480
fps = 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, x_resolution, y_resolution, rs.format.z16, fps)
config.enable_stream(rs.stream.color, x_resolution, y_resolution, rs.format.bgr8, fps)

pipe_profile = pipeline.start(config)
device = pipe_profile.get_device()
depth_sensor = device.query_sensors()[0]
laser_pwr = depth_sensor.get_option(rs.option.laser_power)
set_laser = 360
depth_sensor.set_option(rs.option.laser_power, set_laser)
laser_pwr2 = depth_sensor.get_option(rs.option.laser_power)
print('LASER POWER SETTED from %1.0f' %laser_pwr , 'to %1.0f' %laser_pwr2)
pipeline.stop()

# Start streaming
while not rospy.is_shutdown():

    sensor_start = start_sensor()
    end = rospy.wait_for_message('/close_nodes', String)
    if end.data == 'CLOSE':
        sys.exit()
    print("NODE IS NOW OPEN")
    print("WAITING FOR THE MESSAGE TO START THE SENSOR")
    sensor_start = rospy.wait_for_message('sensor_start',start_sensor)
    
    if sensor_start.start==1:
       
        print("MESSAGE RECEIVED: Publishing Octomap")
        pipe_profile = pipeline.start(config)
        curr_frame = 0
        try:
            while True and rospy.is_shutdown()==False:
                msg = 0
                frames = pipeline.wait_for_frames()
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()
                if not depth_frame or not color_frame:
                    continue
        
                depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
                color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
                depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                align = rs.align(rs.stream.color)
                frameset = align.process(frames)
                colorizer = rs.colorizer()
                aligned_depth_frame = frameset.get_depth_frame()
                colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
          
                pc = rs.pointcloud()
                pc.map_to(color_frame)
                points = pc.calculate(aligned_depth_frame)
                vtx = np.asanyarray(points.get_vertices())
                x = vtx['f0']
                y = vtx['f1']
                z = vtx['f2']
                my_pointcloud = np.vstack((x,y,z))
                pcdx = np.transpose(my_pointcloud)
        
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(pcdx)
                downpcd = pcd.voxel_down_sample(voxel_size=0.1)
                pb_pointcloud = np.asarray(downpcd.points)
                #pb_pointcloud = np.asarray(pcd.points)
                    
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now() 
                header.frame_id = ref_sys
                scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pb_pointcloud)
                pcl_pub.publish(scaled_polygon_pcl)
                
                
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                # images = np.hstack((color_image, colorized_depth))
                cv2.imshow('RealSense', colorized_depth)

                rospy.Subscriber('sensor_stop', stop_sensor, callback)
                
                if cv2.waitKey(1) == 27 or msg==1:
                    print("SENSOR STOPPED")
                    print('--------------------------------------------------------------')
                    break
                curr_frame += 1
        finally:
                pipeline.stop()
                cv2.destroyAllWindows()
    

