#!/home/marco/anaconda3/envs/tensorflow_cpu/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 10:02:22 2020

@author: marco
"""

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

# import tf2_ros
# import tf2_geometry_msgs #import the packages first


Classifier_name = "cascade_hand.xml"
path = "/home/marco/ws_moveit/src/ai_moveit_mp/doc/surface_matching/samples/Classifier/"  # PATH OF THE CASCADE
filename = "Traffic.mp4"
path_file = path+filename
path = path+Classifier_name
overlapThresh= 0.1

pcl_pub = rospy.Publisher("/camera/depth_registered/points", PointCloud2, queue_size =10) #/pointcloud_visualizer

def empty(a):
    pass

def callback(stop_sensor):
    if stop_sensor.stop ==1:
        pipeline.stop()
        cv2.destroyAllWindows()



rospy.init_node('pub_POINTCLOUD')

x_resolution = 640
y_resolution = 480
fps = 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, x_resolution, y_resolution, rs.format.z16, fps)
config.enable_stream(rs.stream.color, x_resolution, y_resolution, rs.format.bgr8, fps)
print("WAITING FOR THE MESSAGE TO START THE SENSOR")
# Start streaming
while not rospy.is_shutdown():
    sensor_start = start_sensor()
    sensor_start = rospy.wait_for_message('sensor_start',start_sensor)
    if sensor_start.start==1:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        print("MESSAGE RECEIVED: Publishing Octomap")
        pipe_profile = pipeline.start(config)
        curr_frame = 0
        try:
            while True and rospy.is_shutdown()==False:
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
        
                # PER PUBBLICARE OCTOMAP
                
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now() 
                header.frame_id = 'panda_hand'
                scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pb_pointcloud)
                pcl_pub.publish(scaled_polygon_pcl)
        
                images = np.hstack((color_image, colorized_depth))
                cv2.imshow('RealSense', colorized_depth)

                rospy.Subscriber('sensor_stop', stop_sensor, callback)
                
                if cv2.waitKey(1) == 27:
                    break
                curr_frame += 1
        finally:
                pipeline.stop()
                cv2.destroyAllWindows()
    
    if sensor_start.start==0:
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        print("Publishing PointCloud")
        pipe_profile = pipeline.start(config)
        curr_frame = 0
        try:
            while True and rospy.is_shutdown()==False:
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
                o3d.io.write_point_cloud("/home/marco/catkin_ws/src/beginner_tutorials/scripts/sensor_pc.ply", downpcd)
                pb_pointcloud = np.asarray(downpcd.points)
        
                header = std_msgs.msg.Header()
                header.stamp = rospy.Time.now() 
                header.frame_id = 'panda_hand'
                scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pb_pointcloud)
                pcl_pub.publish(scaled_polygon_pcl)

                images = np.hstack((color_image, colorized_depth))
                cv2.imshow('RealSense', colorized_depth)
                rospy.Subscriber('sensor_stop', stop_sensor, callback)
                
                if cv2.waitKey(1) == 27:
                    break
                curr_frame += 1
        finally:
                pipeline.stop()
                cv2.destroyAllWindows()

