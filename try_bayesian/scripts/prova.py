#!/home/loris/anaconda3/envs/py3_prova/bin/python

import rospy
import open3d as o3d
import numpy as np
import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import coord_msg_checked
import sys
from std_msgs.msg import Float32MultiArray
import cv2 as cv
import matplotlib.pyplot as plt

global fatt_1

fatt_1=[]
global j
j = 0



if __name__ == '__main__':    
    
    msg_from_planner = coord_msg_checked()
    pcd = o3d.io.read_point_cloud("/home/loris/ply_and_stl/scene_and_model/Assieme_con_raccordi.ply")
    pcd1 = o3d.io.read_point_cloud("/home/loris/ply_and_stl/scene_and_model/Assieme_con_raccordi.ply")


    print('sono nel main prima del while')

    rospy.init_node('open3d_node', anonymous=True)
    pub = rospy.Publisher('halcon_consensus', coord_msg_checked, queue_size=10)
    while not rospy.is_shutdown():
        
        print("sono nel while")
        print("j=",j)

        msg_from_planner = rospy.wait_for_message("/coord_msg_checked",coord_msg_checked)

        #camera_data = rospy.wait_for_message("halcon_finish_message", String)
        #print("\n\n ------------------------------------------\n\n")
        print("ho appena ricevuto",msg_from_planner.x)
        print("\nsono nel while e ho ricevuto il messaggio da halcon, invio una nuova posizione\n")
        

        camera_x = msg_from_planner.x
        camera_y = msg_from_planner.y
        camera_z = msg_from_planner.z
        

        
        #rospy.init_node('talker', anonymous=True)

        print("hello")

        print(pcd)
        camera = [camera_x/1000, camera_y/1000, camera_z/1000]
        diameter = np.linalg.norm(np.asarray(pcd.get_max_bound()) - np.asarray(pcd.get_min_bound()))
        print(diameter)

        radius = diameter * 1000

        print("Get all points that are visible from given view point")
        _, pt_map = pcd.hidden_point_removal(camera, radius)
        pcd_cut = pcd.select_by_index(pt_map)
        
        #plotto la nuvola
        print(camera_x/1000,camera_y/1000,camera_z/1000)

        downpcd = pcd.voxel_down_sample(voxel_size=.05)
        xyz_load = np.asarray(downpcd.points)
        print('xyz_load')
        print(xyz_load)
        print(len(xyz_load))
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')




        ax.scatter(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2])
        ax.scatter(camera_x/1000,camera_y/1000,camera_z/1000, c = 'r',)
        
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        print("sono qui")
        plt.show()
        #finito di plottare
        o3d.io.write_point_cloud("/home/loris/ply_and_stl/scene_and_model/Scena_Fusoliera_tagliata.ply",pcd_cut)
        rospy.loginfo("nuvola salvata")
        #cloud_points = cv.ppf_match_3d.loadPLYSimple("/home/loris/Scaricati/dino_o3d.ply",1)
        print("\npremere per avviare messaggio analisi finita")
        #input()
        
        msg_to_halcon = msg_from_planner
        rospy.loginfo("\n messaggio pubblicato \n")
        pub.publish(msg_to_halcon)
      
        print("\n x= %f y= %f z = %f check = %i",msg_to_halcon.x,msg_to_halcon.y,msg_to_halcon.z,msg_to_halcon.check)
        rospy.sleep(1)
        j=1
        



