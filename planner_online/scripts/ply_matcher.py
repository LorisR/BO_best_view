#!/home/marco/anaconda3/envs/tensorflow_cpu/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 30 15:41:33 2020

@author: marco
"""
import numpy as np
import sys
import rospy
from planner_online.msg import coord_to_reach
from std_msgs.msg import String
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import cv2 as cv

#CHANGE ROOT
root = '/home/marco'

rospy.init_node('Ply_Matcher', anonymous=True)
print("PLY MATCHER NODE\n")
pubesc = rospy.Publisher('/close_nodes', String, queue_size =10)
print('WAITING FOR THE MESSAGE TO INITIALIZE PLY ANALISYS')
rospy.wait_for_message('ply_matcher', String)

txt = root+'/catkin_ws/src/planner_online/Pointcloud_Acquired/position_list.txt'
coord_quat = (np.loadtxt(txt, 'float32'))
print('FILE READED\n')
pcd2 = o3d.geometry.PointCloud()

pp=0

while pp<len(coord_quat):
    
    pos = np.matrix(([coord_quat[pp,0]],
                     [coord_quat[pp,1]],
                     [coord_quat[pp,2]]), 'float32')              
    quat = coord_quat[pp,3:]
    r = R.from_quat(quat)
    rot_mat = np.matrix(r.as_matrix(), 'float32')
    hom = np.matrix(([0,0,0, 1]), 'float32')
    pos1 = np.hstack((rot_mat, pos))
    pose = np.vstack((pos1, hom))
    
    num = pp+1
    num = '%d' %num
    ply = root+'/catkin_ws/src/planner_online/Pointcloud_Acquired/ply_scan_'+num+'.ply'
    pcd = o3d.io.read_point_cloud(ply)
    scene_points = np.asarray(pcd.points, 'float32') 
    
    scene_t = cv.ppf_match_3d.transformPCPose(scene_points, pose)
    points1 = scene_t
    if pp==0:
        points = points1
    if pp>0:
        points = np.vstack((points, points1))
        
    cc=pp+1
    print('PLY %d ELABORATED' %cc)
    print('---------------------------------------\n')
    pp=pp+1

print('\nDOWNSAMPLING AND SAVING THE TOTAL POINTCLOUD')
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(points)
downpcd = pcd2.voxel_down_sample(voxel_size=0.01)
ply = root+'/catkin_ws/src/planner_online/Pointcloud_Acquired/TOTAL_scan.ply'
o3d.io.write_point_cloud(ply,downpcd) 
downsample_pc = np.asarray(downpcd.points)    
print('\nCLOSING ALL NODES')
pubesc.publish('CLOSE')
#rospy.sleep(3)
sys.exit