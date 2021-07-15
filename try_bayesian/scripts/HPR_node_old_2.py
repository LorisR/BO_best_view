#!/home/loris/anaconda3/bin/python


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
import re
import time

global fatt_1

fatt_1=[]
global j
j = 0

def save_view_point(pcd, filename):
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=800, height=600)
    vis.add_geometry(pcd)
    vis.run()  # user changes the view and press "q" to terminate
    param = vis.get_view_control().convert_to_pinhole_camera_parameters()
    o3d.io.write_pinhole_camera_parameters(filename, param)
    vis.destroy_window()

def load_view_point(pcd, filename):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    ctr = vis.get_view_control()
    param = o3d.io.read_pinhole_camera_parameters(filename)
    vis.add_geometry(pcd)
    ctr.convert_from_pinhole_camera_parameters(param)
    vis.get_render_option().load_from_json("/home/loris/ply_and_stl/json_o3d/hpr.json")
    vis.run()
    vis.destroy_window()

    
def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring


if __name__ == '__main__':    
    k=0
    #lettura dei parametri 
    
    param_file_name = "//home/loris/ply_and_stl/param.txt"
    
    file = open(param_file_name,"r")
    parameters = file.readlines()
    scena_tagliata_file_name = taglia_stringa("{(.*?)}",parameters[28],"testo") 
    modelname  = taglia_stringa("{(.*?)}",parameters[0],"testo") 
    hpr_radius   = taglia_stringa("{(.*?)}",parameters[20],"numero") 

    msg_from_planner = coord_msg_checked()
    pcd = o3d.io.read_point_cloud(modelname)
    pcd1 = o3d.io.read_point_cloud(modelname)
    pcd_point = o3d.geometry.PointCloud()



    rospy.init_node('open3d_node', anonymous=True)
    pub = rospy.Publisher('halcon_consensus', coord_msg_checked, queue_size=10)
    while not rospy.is_shutdown():
        
        msg_from_planner = rospy.wait_for_message("/coord_msg_checked",coord_msg_checked)

        print("ho appena ricevuto: \nx =\t",msg_from_planner.x,"\ny =\t",msg_from_planner.y,"\nz =\t ",msg_from_planner.z)
        print("\nho ricevuto il messaggio dal planner, invio una nuova posizione\n")

        #organize the viewpoint coordinates
        camera_x = msg_from_planner.x
        camera_y = msg_from_planner.y
        camera_z = msg_from_planner.z
        camera = [camera_x, camera_y, camera_z]
        #create a sphere for visualization of the viewpoint
        mesh = o3d.geometry.TriangleMesh.create_sphere(radius =.015,resolution=10)
        mesh.translate(camera,relative = False)
        mesh.paint_uniform_color([1, 0.706, 0])
        #camera = [0, 0, 0]

        print("Get all points that are visible from given view point")
        _, pt_map = pcd.hidden_point_removal(camera, hpr_radius)
        pcd_cut = pcd.select_by_index(pt_map)
        pcd_cut.paint_uniform_color([.7, 0.7, 0.7])

        # xecute this function in order to save a new visualizer viewpoint
        # save_view_point(pcd_cut, "/home/loris/ply_and_stl/json_o3d/hpr_view.json")

        #plotto utilizzando open3d
      
        if j==0:    
            vis = o3d.visualization.Visualizer()
            vis.create_window(width = 800,height =600)

            vis.add_geometry(pcd_cut)
            vis.add_geometry(pcd_point)
            vis.add_geometry(mesh)
            param = o3d.io.read_pinhole_camera_parameters("/home/loris/ply_and_stl/json_o3d/hpr_view.json")
            ctr = vis.get_view_control()
            ctr.convert_from_pinhole_camera_parameters(param)
            vis.get_render_option().load_from_json("/home/loris/ply_and_stl/json_o3d/hpr.json")

    

            vis.update_geometry(pcd_cut)
            vis.update_geometry(pcd_point)   
            vis.update_geometry(mesh)  
            vis.poll_events()
            vis.update_renderer()

        #save the cutted pointcloud

        o3d.io.write_point_cloud(scena_tagliata_file_name,pcd_cut)
        rospy.loginfo("nuvola salvata")
        
        msg_to_halcon = msg_from_planner
        pub.publish(msg_to_halcon)
        rospy.loginfo("\n messaggio pubblicato \n")
      
        print("\n x= %f y= %f z = %f check = %i",msg_to_halcon.x,msg_to_halcon.y,msg_to_halcon.z,msg_to_halcon.check)
        
        #update the visualization in order to keep the window fresh
        for kj in range(5):
            vis.clear_geometries()       
            vis.add_geometry(pcd_cut,False)
            vis.add_geometry(pcd_point,False)
            vis.add_geometry(mesh,False)
            vis.update_geometry(pcd_cut)
            vis.update_geometry(pcd_point)   
            vis.update_geometry(mesh)            
            vis.poll_events()
            vis.update_renderer()
            time.sleep(.15)

        j=1



