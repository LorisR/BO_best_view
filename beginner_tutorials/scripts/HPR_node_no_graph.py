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
import re

global fatt_1

fatt_1=[]
global j
j = 0


def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring


if __name__ == '__main__':    
    
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


    print('sono nel main prima del while')

    rospy.init_node('open3d_node', anonymous=True)
    pub = rospy.Publisher('halcon_consensus', coord_msg_checked, queue_size=10)
    while not rospy.is_shutdown():
        
        print("sono nel while")
        print("j=",j)

        msg_from_planner = rospy.wait_for_message("/coord_msg_checked",coord_msg_checked)

        #camera_data = rospy.wait_for_message("halcon_finish_message", String)
        #print("\n\n ------------------------------------------\n\n")
        print("ho appena ricevuto: \tx = ",msg_from_planner.x,"\ty = ",msg_from_planner.y,"\tz = ",msg_from_planner.z)
        print("\nho ricevuto il messaggio da halcon, invio una nuova posizione\n")
        

        camera_x = msg_from_planner.x
        camera_y = msg_from_planner.y
        camera_z = msg_from_planner.z
        

        
        #rospy.init_node('talker', anonymous=True)

        print("hello")

        print(pcd)
        camera = [camera_x, camera_y, camera_z]


        print("Get all points that are visible from given view point")
        _, pt_map = pcd.hidden_point_removal(camera, hpr_radius)
        pcd_cut = pcd.select_by_index(pt_map)
        
        #plotto la nuvola
        print(camera_x,camera_y,camera_z)

        #finito di plottare
        o3d.io.write_point_cloud(scena_tagliata_file_name,pcd_cut)
        rospy.loginfo("nuvola salvata")
        #cloud_points = cv.ppf_match_3d.loadPLYSimple("/home/loris/Scaricati/dino_o3d.ply",1)
        
        msg_to_halcon = msg_from_planner
        pub.publish(msg_to_halcon)
        rospy.loginfo("\n messaggio pubblicato \n")
      
        print("\n x= %f y= %f z = %f check = %i",msg_to_halcon.x,msg_to_halcon.y,msg_to_halcon.z,msg_to_halcon.check)
        rospy.sleep(1)
        j=1
        



