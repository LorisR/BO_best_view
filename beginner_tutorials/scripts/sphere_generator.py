#!/home/loris/anaconda3/envs/py3/bin/python
# #%% utile per plottare una finestra di debug

from mpl_toolkits import mplot3d
import numpy as np
import math
import matplotlib.pyplot as plt 
import cv2 as cv
import rospy
from std_msgs.msg import String
from beginner_tutorials.msg import o3d_coord_msg
from beginner_tutorials.msg import prova_msg
from math import pi
import re
from scipy.spatial.transform import Rotation as R
def eul2rotm(phi,theta,gamma):
    rz = np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
    ry = np.matrix([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    rx = np.matrix([[1,0,0],[0,np.cos(gamma),-np.sin(gamma)],[0,np.sin(gamma),np.cos(gamma)]])
    rotm = rz*ry*rx
    return rotm

def taglia_stringa(pattern,stringa,tipo):
        ''' se il valore da leggere è una stringa mettere testo altrimenti numero'''
        substring = re.search(pattern, stringa).group(1)
        if tipo =="numero":
            return float(substring)
        elif tipo == "testo":
            return substring

    
 
if __name__ == '__main__':
    
    #leggo parametri da testo
    
    param_file_name = "/home/loris/ply_and_stl/param.txt"
    file = open(param_file_name,"r")
    parameters = file.readlines()
    modelname  = taglia_stringa("{(.*?)}",parameters[0],"testo") 
    center_x   = taglia_stringa("{(.*?)}",parameters[1],"numero") 
    center_y   = taglia_stringa("{(.*?)}",parameters[2],"numero")
    center_z   = taglia_stringa("{(.*?)}",parameters[3],"numero")
    rho        = taglia_stringa("{(.*?)}",parameters[4],"numero")
    lat_min    = taglia_stringa("{(.*?)}",parameters[5],"numero")
    lat_max    = taglia_stringa("{(.*?)}",parameters[6],"numero")
    n_lat      = taglia_stringa("{(.*?)}",parameters[7],"numero")
    long_min    = taglia_stringa("{(.*?)}",parameters[8],"numero")
    long_max    = taglia_stringa("{(.*?)}",parameters[9],"numero")
    n_long      = taglia_stringa("{(.*?)}",parameters[10],"numero")    
    cloud_points = cv.ppf_match_3d.loadPLYSimple(modelname,1)
    
  

    
    lat_min_r = lat_min/180*pi
    lat_max_r = lat_max/180*pi
    long_min_r = long_min/180*pi
    long_max_r = long_max/180*pi
    
    n_df = (lat_max_r-lat_min_r)/n_lat
    m_df = (long_max_r-long_min_r)/n_long
    phi = np.arange(lat_min_r, lat_max_r, n_df)
    theta = np.arange(long_min_r, long_max_r, m_df)
    matrix = np.zeros((9,len(phi)*len(theta)),'float64')
    sample_points = np.zeros((len(phi)*len(theta),3), dtype="float64")
    a=0
   # mat2 = eul2rotm(pi/2,0,-pi/2)
    
    #points calculation
    for i in range(0, len(phi)):
        for pp in range(0, len(theta)):
            #PHI VA DA 0 A 180 GRADI
            #THETA VA DA 0 A 360
            sample_points[a,0] = rho*np.sin(theta[pp])*np.cos(phi[i])
            sample_points[a,1] = rho*np.sin(theta[pp])*np.sin(phi[i])
            sample_points[a,2] = rho*np.cos(theta[pp])
            
            rotm_sphere = np.matrix(([np.sin(theta[pp])*np.cos(phi[i]), np.sin(theta[pp])*np.sin(phi[i]), np.cos(theta[pp])],
                                     [np.cos(theta[pp])*np.cos(phi[i]), np.cos(theta[pp])*np.sin(phi[i]), -np.sin(theta[pp])],
                                     [-np.sin(phi[i]), np.cos(phi[i]), 0]))
        
            rotm_sphere1=rotm_sphere
            ux = rotm_sphere1[1,0]
            uy = rotm_sphere1[1,1]
            uz = rotm_sphere1[1,2]
            
            gamma = -pi/2
            com = 1-np.cos(gamma)
            ct = np.cos(gamma)
            st = np.sin(gamma)
            #DERIVE FROM EULER THEOREM
            #http://www.ladispe.polito.it/corsi/meccatronica/01GTG/2008-09/Slides/Rotazioni.pdf
            rtp_camera = np.matrix(([ux*ux*com+ct, ux*uy*com-uz*st, ux*uz*com+uy*st],
                                    [ux*uy*com+uz*st, uy*uy*com+ct, uy*uz*com-ux*st],
                                    [ux*uz*com-uy*st, uy*uz*com+ux*st, uz*uz*com+ct]))
            # r = R.from_matrix(rtp_camera)
            # quate = np.array(r.as_quat(), 'float')
            
            mat = rtp_camera*(rotm_sphere.T)
            r = R.from_matrix(mat)
            quat = np.array(r.as_quat(), 'float')
                        
            mat_vet = np.reshape(mat,-1)
            matrix[:,a] =  mat_vet
            a = a+1
        
    x = sample_points[:,0] + center_x
    y = sample_points[:,1] + center_y
    z = sample_points[:,2] + center_z
    sample_points[:,0] = x
    sample_points[:,1] = y
    sample_points[:,2] = z

   
    #sample_points_vector = np.reshape(sample_points, -1)
    

    
    
    #sample_points_vector2 = np.reshape(sample_points_vector, (-1,6))
    print("\n-------------------------------------------------------------------------\n")
    print(len(sample_points))
    print("generazione della sfera terminata, avvio la prima coordinata\n")
    j=0
    i=0
    rospy.sleep(4)

    pub1 = rospy.Publisher("o3d_coord_msg", o3d_coord_msg, queue_size=100)
    rospy.init_node('sphere_coordinates', anonymous=True)
    msg_to_o3d = o3d_coord_msg()
    
    while (rospy.is_shutdown()==False and i<len(sample_points)):


        print(" sono nel while")
        if(j==1):
            
            camera_data = rospy.wait_for_message("halcon_finish_message", String)
            print("\n\n ------------------------------------------\n\n")
            print("ho appena ricevuto",camera_data.data)
            print("\nsono nel while e ho ricevuto il messaggio da halcon, invio una nuova posizione\n")

        msg_to_o3d.x = sample_points[i,0]
        
        msg_to_o3d.y = sample_points[i,1]
        
        msg_to_o3d.z = sample_points[i,2]
        #matrice di rotazione
        msg_to_o3d.a1 = matrix[0,i]
        msg_to_o3d.a2 = matrix[1,i]
        msg_to_o3d.a3 = matrix[2,i]
        msg_to_o3d.a4 = matrix[3,i]
        msg_to_o3d.a5 = matrix[4,i]
        msg_to_o3d.a6 = matrix[5,i]
        msg_to_o3d.a7 = matrix[6,i]
        msg_to_o3d.a8 = matrix[7,i]
        msg_to_o3d.a9 = matrix[8,i]


        rospy.sleep(1)
        pub1.publish(msg_to_o3d)
        print("sto per pubblicare")
        
        

        rospy.loginfo(msg_to_o3d)

        i=i+1     
        j=1
        


