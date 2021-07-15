#!/home/marco/anaconda3/envs/tensorflow_cpu/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Sep  4 10:02:22 2020

@author: marco
"""

import pyrealsense2 as rs
import open3d as o3d
import numpy as np
import cv2
import os
import matplotlib.pyplot as plt 
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import rospy
import sensor_msgs.point_cloud2 as pcl2
from planner_online.msg import start_sensor
from planner_online.msg import stop_sensor
from std_msgs.msg import String
global msg
import sys

#CHANGE ROOT
root = '/home/marco'

Classifier_name = "cascade_hand.xml"
path = root+"/catkin_ws/src/planner_online/Classifier/"  # PATH OF THE CASCADE
filename = "Traffic.mp4"
path_file = path+filename
path = path+Classifier_name
overlapThresh= 0.1
pcl_pub = rospy.Publisher("/pointcloud_visualizer", PointCloud2, queue_size =1000)
obj = rospy.Publisher("/object_found", String, queue_size =1000)

rospy.init_node('pub_POINTCLOUD')
print("POINTCLOUD NODE\n")
cascade = cv2.CascadeClassifier(path)

ref_sys = 'panda_hand'
ref_sys = 'Camera'

x_resolution = 640
y_resolution = 480
fps = 30

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, x_resolution, y_resolution, rs.format.z16, fps)
config.enable_stream(rs.stream.color, x_resolution, y_resolution, rs.format.bgr8, fps)


# device = pipe_profile.get_device()
# depth_sensor = device.query_sensors()[0]
# laser_pwr = depth_sensor.get_option(rs.option.laser_power)
# set_laser = 360
# depth_sensor.set_option(rs.option.laser_power, set_laser)
# laser_pwr2 = depth_sensor.get_option(rs.option.laser_power)
# print('LASER POWER SETTED from %1.0f' %laser_pwr , 'to %1.0f' %laser_pwr2)
curr_frame = 0
distance_limit =1.0

def empty(a):
    pass

def nms(boxes, overlapThresh): #non maximum suppression

 	if len(boxes) == 0:
	   return []
    
 	pick = []
 	x1 = boxes[:,0]
 	y1 = boxes[:,1]
 	x2 = boxes[:,0]+boxes[:,2]
 	y2 = boxes[:,1]+boxes[:,3]
 	area = (x2 - x1 + 1) * (y2 - y1 + 1)
 	idxs = np.argsort(y2)
     
 	while len(idxs) > 0:  
        
         last = len(idxs) - 1
         i = idxs[last]
         pick.append(i)
         suppress = [last]
         for pos in range(0, last):
            j = idxs[pos]
            xx1 = max(x1[i], x1[j])
            yy1 = max(y1[i], y1[j])
            xx2 = min(x2[i], x2[j])
            yy2 = min(y2[i], y2[j])
            w = max(0, xx2 - xx1 + 1)
            h = max(0, yy2 - yy1 + 1)
            overlap = float(w * h) / area[j]
            if overlap > overlapThresh:
                suppress.append(pos)
         idxs = np.delete(idxs, suppress)
 	return boxes[pick]
 
def callback(stop_sensor):
    if stop_sensor.stop==1:
        global msg
        msg=1



while not rospy.is_shutdown():
    sensor_start = start_sensor()
    print("WAITING FOR THE MESSAGE TO START THE SENSOR")
    end = rospy.wait_for_message('/close_nodes', String)
    if end.data == 'CLOSE':
        sys.exit()
    sensor_start = rospy.wait_for_message('sensor_start',start_sensor)

    if sensor_start.start==0:
        # fig = plt.figure()
        # fig.set_size_inches(10, 8)
        pipe_profile = pipeline.start(config)
        print("\nMESSAGE RECEIVED: Publishing Pointcloud")
        print("\nMAXIMUM POINT DISTANCE SETTED: %1.1f" %distance_limit, 'm')
        curr_frame = 0
        zeros = np.array((0,0,0,0,0,0,0,0,0,0), 'float32')
        bb =0
        try: 
            while True and rospy.is_shutdown()==False:
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.createTrackbar("Scale","RealSense",60,1000,empty)
                cv2.createTrackbar("Neig","RealSense",40,60,empty)
                cv2.createTrackbar("Min Area","RealSense",20000,100000,empty)
                cv2.createTrackbar("Pointcloud Sampled","RealSense",60,200,empty)
                
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
        
                gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
                
                scaleVal = 1 + (cv2.getTrackbarPos("Scale", "RealSense") /1000)
                neig=cv2.getTrackbarPos("Neig", "RealSense")
                pc_area = cv2.getTrackbarPos("Pointcloud Sampled", "RealSense")
                objects = cascade.detectMultiScale(gray,scaleVal, neig) 
                boxes = objects
                boxes = nms(objects, overlapThresh)
                areaTot = 0
                
                if len(boxes)<1:
                    zeros = np.array((0,0,0,0,0,0,0,0,0,0), 'float32')
                    
                if len(boxes)>0 and curr_frame> 20:
                    for i in range(0, len(boxes)):
                            x = boxes[i,0]
                            y = boxes[i,1]
                            w = boxes[i,2]#-boxes[i,0]
                            h = boxes[i,3]#-boxes[i,1]
                            xn = x-int(w*(pc_area/100))
                            yn = y-int(h*(pc_area/100))
                            wpc = int(w*(1+2*(pc_area/100)))
                            hpc = int(h*(1+2*(pc_area/100)))
                            area = wpc*hpc
                            areaTot = areaTot + area
                            minArea = cv2.getTrackbarPos("Min Area", "RealSense")
                            if area >minArea:
                                cv2.rectangle(color_image,(x,y),(x+w,y+h),(0,0,255),2)
                                cv2.rectangle(color_image,(xn,yn),(xn+wpc,yn+hpc),(0,255,0),2)
                                cv2.putText(color_image,'Hand' ,(x,y-5),cv2.FONT_HERSHEY_PLAIN , 1,(0,0,255),2)
                                cv2.putText(color_image,'PC Sampled' ,(xn,yn+15+hpc),cv2.FONT_HERSHEY_PLAIN , 1,(0,255,0),2)
                    
                    if bb>=10:
                        zeros = np.array((0,0,0,0,0,0,0,0,0,0), 'float32')
                        bb=0
                    zeros[bb]=1
                    bb= bb+1
                    
                    pc = rs.pointcloud()
                    pc.map_to(color_frame)
                    points = pc.calculate(aligned_depth_frame)
                    vtx = np.asanyarray(points.get_vertices())
                    x = vtx['f0']
                    y = vtx['f1']
                    z = vtx['f2']
                    xx = np.reshape(x,(y_resolution,x_resolution))
                    yy = np.reshape(y,(y_resolution,x_resolution))
                    zz = np.reshape(z,(y_resolution,x_resolution))
    
                    for b in range(0, len(boxes)):
                        
                        x = boxes[b,0]
                        y = boxes[b,1]
                        w = boxes[b,2]
                        h = boxes[b,3]
                        xn = x-int(w*(pc_area/100))
                        yn = y-int(h*(pc_area/100))
                        wpc = int(w*(1+2*(pc_area/100)))
                        hpc = int(h*(1+2*(pc_area/100)))
                        if xn<0:
                              xn = 0
                        if yn<0:
                            yn = 0
                        if xn+wpc>x_resolution:
                            wpc = x_resolution-xn
                        if yn+hpc>y_resolution:
                            hpc = y_resolution-yn
                        if b==0:
                            xxx = xx[yn:yn+hpc,xn:xn+wpc]
                            yyy = yy[yn:yn+hpc,xn:xn+wpc]
                            zzz = zz[yn:yn+hpc,xn:xn+wpc]
                             
                            xxx = np.reshape(xxx,-1)
                            yyy = np.reshape(yyy,-1)
                            zzz = np.reshape(zzz,-1)
                            
                            pc_cutted = np.vstack((xxx,yyy,zzz))
                            pc_cutted = pc_cutted.T
    
                            index=[]
                            for a in range(0, len(pc_cutted)-1):
                                if pc_cutted[a,2]<distance_limit and pc_cutted[a,2]>0.2 :
                                      index.append(a)
                                      
                            index = np.asarray(index, 'int')
                            pcdx_cutted = pc_cutted[index,:]
                            
                            pcd = o3d.geometry.PointCloud()
                            pcd.points = o3d.utility.Vector3dVector(pcdx_cutted)
                            downpcd = pcd.voxel_down_sample(voxel_size=0.002)
                            o3d.io.write_point_cloud(root+
                                        "/catkin_ws/src/planner_online/ply_sensor/sensor_pc.ply", downpcd)
                            downpcd = np.asarray(downpcd.points, 'float32')
                            
                            if zeros[0]==1 and zeros[1]==1 and zeros[2]==1 and zeros[3]==1 and zeros[4]==1 and zeros[5]==1 and zeros[6]==1 and zeros[7]==1 and zeros[8]==1:
                                obj.publish('OBJECT FOUND')
                                print('OBJECT RECOGNIZED')
                        
                            # header = std_msgs.msg.Header()
                            # header.stamp = rospy.Time.now() 
                            # header.frame_id = ref_sys
                            # scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, pcdx_cutted)
                            # pcl_pub.publish(scaled_polygon_pcl)
                                            
                                        
                images = np.hstack((color_image, colorized_depth))
                rospy.Subscriber('sensor_stop', stop_sensor, callback)
                cv2.imshow('RealSense', images)
                rospy.Subscriber('sensor_stop', stop_sensor, callback)
            
                if cv2.waitKey(1) == 27 or msg==1:
                    print("SENSOR STOPPED")
                    break
                curr_frame += 1
        finally:
                pipeline.stop()
                cv2.destroyAllWindows()