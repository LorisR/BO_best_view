#!/home/loris/anaconda3/envs/py3/bin/python
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
import re
from tkinter import filedialog
from tkinter import *



#CHANGE ROOT
root = '/home/loris'
param_file_name = root +"/ply_and_stl/param.txt"

def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring

file = open(param_file_name,"r")
parameters = file.readlines()
offline_dir = taglia_stringa("{(.*?)}",parameters[37],"testo")
#apro finestra di dialogo
root = Tk()
root.filename =  filedialog.askopenfilename(initialdir = offline_dir,title = "Select file",filetypes = (("txt files","*.txt"),("all files","*.*")))
best_view_file = root.filename
root.destroy()
print (root.filename)

rospy.init_node('READER', anonymous=True)
print("READER NODE\n")
pub = rospy.Publisher('/position_and_orientations', coord_to_reach, queue_size =10)
pubesc = rospy.Publisher('/close_nodes', String, queue_size =10)
rospy.sleep(1)
filex = (np.loadtxt(best_view_file, 'float32'))
filex = filex[:,0:9]
file1 = filex[filex[:,8].argsort()]
pose_coord = np.flip(file1,0)
coord = coord_to_reach()
j=0
ii=1
rospy.sleep(5)
i=0
pubesc.publish('OPEN')
# rospy.sleep(1)
print("FILE READED: STARTING PUBLISHING COORDINATES")
while not rospy.is_shutdown() and ii<=len(pose_coord):
    for i in range(0, len(pose_coord)):
        pose_coord[i,0] = pose_coord[i,0]
        pose_coord[i,1] = pose_coord[i,1]
        pose_coord[i,2] = pose_coord[i,2]
        print('WAITING FOR THE REQUEST')
        rospy.wait_for_message('Request_New_Coord', String)
        rospy.sleep(3)
        pubesc.publish('OPEN')
        print('REQUEST RECEIVED')
        coord.x = pose_coord[i,0]
        coord.y = pose_coord[i,1]
        coord.z = pose_coord[i,2]
        coord.qx = pose_coord[i,3]
        coord.qy = pose_coord[i,4]
        coord.qz = pose_coord[i,5]
        coord.qw = pose_coord[i,6]
        rospy.sleep(3)
        pub.publish(coord)
        print('\n',ii, 'of', len(pose_coord),' Coordinates and Orientations Published')
        print('xyz: %1.4f' %pose_coord[i,0],
                  '%1.4f' %pose_coord[i,1],
                  '%1.4f' %pose_coord[i,2],
                  'qxyzw: %1.4f' %pose_coord[i,3],
                  '%1.4f' %pose_coord[i,4],
                  '%1.4f' %pose_coord[i,5],
                  '%1.4f' %pose_coord[i,6])
            
        print('--------------------------------------------------------------')
        ii = ii+1
        
    if ii==(len(pose_coord)+1):
        print('WAITING FOR FINISHED MATCHING')
        rospy.wait_for_message('Request_New_Coord', String)
        rospy.sleep(3)
        pubesc.publish('CLOSE')
        print('All coordinates has been sent')
        print('CLOSING ALL NODES')
        rospy.sleep(3)
        sys.exit()
