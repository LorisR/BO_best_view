#!/home/loris/anaconda3/envs/py3/bin/python

import cv2 as cv
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt 
import rospy

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2


import open3d as o3d
import numpy as np
import sys
import time
import re
import copy
from std_msgs.msg import String
from beginner_tutorials.msg import coord_msg_checked
from planner_online.msg import coord_to_reach
from scipy.spatial.transform import Rotation as R
from planner_online.msg import start_sensor
from planner_online.msg import stop_sensor
from planner_online.msg import pose
from planner_online.msg import coord_msg_online

def to_rad(ang):
    from math import pi
    ang_conv = ang/180*pi
    return ang_conv

def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring

#CHANGE ROOT
param_file_name = "/home/loris/ply_and_stl/param.txt"
file = open(param_file_name,"r")
parameters = file.readlines()


scenename = taglia_stringa("{(.*?)}",parameters[48],"testo") 
scenename_t = taglia_stringa("{(.*?)}",parameters[49],"testo") 
json_name = "/home/loris/ply_and_stl/json_o3d/planner_online.json"

vis = o3d.visualization.Visualizer()
pinza_filename = taglia_stringa("{(.*?)}",parameters[27],"testo")
pcd_model = o3d.io.read_point_cloud(pinza_filename)
pcd_model.paint_uniform_color([0, 0.502, 1])
hom_matrix = np.zeros((4,4))

if __name__ == '__main__':
      
        rospy.init_node('Pose_Estimation_Algorithm', anonymous=True)
        messaggio_ricevuto = coord_msg_checked()
        pinza_coord = pose()
        msg_to_halcon = coord_msg_online()
        print('POSE ESTIMATOR NODE \n')
        # pub = rospy.Publisher('Matching_Finished', String, queue_size=10)
        pub = rospy.Publisher('halcon_consensus', coord_msg_online, queue_size=10)
        pub2 = rospy.Publisher('Request_New_Coord', String, queue_size=10)
        
        #file = np.array((0,0,0,0,0), 'float32')
        j=0
        i=0
        try:
            
            while not rospy.is_shutdown() :
                
                coord = coord_to_reach()
                print('WAITING FOR THE SET OF COORDINATES AND ORIENTATIONS') 
                end = rospy.wait_for_message('/close_nodes', String)
                if end.data == 'CLOSE':
                        sys.exit()
                coord = rospy.wait_for_message('/position_and_orientations', coord_to_reach)
                print('MESSAGE RECEIVED') 
                
                pos = np.matrix(([coord.x],
                                  [coord.y],
                                  [coord.z]), 'float32')
                quat = np.array([coord.qx, coord.qy, coord.qz, coord.qw], 'float32')
                # pos = np.matrix(([1], [1],[ 1]), 'float32')
                # quat = np.array([1, 0, 0, 0], 'float32')
                r = R.from_quat(quat)
                rot_mat = np.matrix(r.as_matrix(), 'float32')
                hom = np.matrix(([0,0,0, 1]), 'float32')
                pos1 = np.hstack((rot_mat, pos))
                pose_hom = np.vstack((pos1, hom))
                
                print('WAITING THE SAVING OF SENSOR POINTCLOUD') 
                rospy.wait_for_message('sensor_stop', stop_sensor)
                print('MESSAGE RECEIVED')
                rospy.sleep(2)
                
                pcd = o3d.io.read_point_cloud("%s" % scenename)
                scene_points = np.asarray(pcd.points, 'float32') 
                scene_t = cv.ppf_match_3d.transformPCPose(scene_points, pose_hom)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(scene_t)
                o3d.io.write_point_cloud(scenename_t,pcd)

                print('TRANSLATED SENSOR POINT CLOUD wrt ORIGIN SAVED') 

                
                msg_to_halcon.x = pos[0]
                msg_to_halcon.y = pos[1]
                msg_to_halcon.z = pos[2]
                msg_to_halcon.qx = quat[0]
                msg_to_halcon.qy = quat[1]
                msg_to_halcon.qz = quat[2]
                msg_to_halcon.qw = quat[3]
                
                pub.publish(msg_to_halcon)
                print("Matching request to Halcon sent")
#                pcd_1 = o3d.io.read_point_cloud(scenename_t)
                
                pinza_coord = rospy.wait_for_message("/halcon_publish_pose",pose)            
                pcd.paint_uniform_color([0.3, 0.3, 0.3])
                vis = o3d.visualization.Visualizer()
                vis.create_window(width = 1366,height =768)
                vis.get_render_option().load_from_json(json_name)
                pcd_model_plot = copy.deepcopy(pcd_model)
                
                model_pos = np.array([pinza_coord.pos_x,pinza_coord.pos_y,pinza_coord.pos_z])
                print(model_pos)
                model_ang = np.array([to_rad(pinza_coord.rot_x),to_rad(pinza_coord.rot_y),to_rad(pinza_coord.rot_z)])
                hom_matrix[0:3,0:3] = o3d.geometry.get_rotation_matrix_from_xyz(model_ang)
                hom_matrix[0:3,3] = model_pos
                hom_matrix[3,3] = 1
                pcd_model_plot.transform(hom_matrix)
                vis.add_geometry(pcd)
                vis.add_geometry(pcd_model_plot)
                vis.run()
                vis.destroy_window()

                
                print('REQUESTED NEW SET OF COORDINATES')
                pub2.publish('REQUESTING NEW SET OF COORDINATES')
                print('--------------------------------------------------------------')

        except rospy.ROSInterruptException():
            pass

 