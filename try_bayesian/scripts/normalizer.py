#!/home/loris/anaconda3/envs/py3/bin/python


import numpy as np
from numpy import pi
from math import sin,cos
import rospy
from beginner_tutorials.msg import o3d_coord_msg
from try_bayesian.msg import bayesian_msg
from scipy.spatial.transform import Rotation as R
import re

def normalizer(coord_to_be_norm, lower_limit, upper_limit):
    m = upper_limit-lower_limit
    coord_norm = m*coord_to_be_norm+lower_limit
    return coord_norm

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

def fromSENSORtoEEF(pos,quat):

    r = R.from_quat(quat)
    rot_mat = np.matrix(r.as_matrix(), 'float')
    hom = np.matrix(([0,0,0, 1]), 'float')
    pos1 = np.hstack((rot_mat, pos))
    pose = np.vstack((pos1, hom))
    
    R1 = np.matrix(([1,0,0,-0.02],
                    [0, 1, 0, 0],
                    [0,0,1,0.05285],
                    [0,0,0,1]), 'float')
    R1_inv = np.linalg.inv(R1)
    
    plan_matrix = pose*R1_inv
    
    r = R.from_dcm(plan_matrix[:3,:3])
    quat2 = np.array(r.as_quat(), 'float')
    pos2 = np.array((plan_matrix[0,3],plan_matrix[1,3],plan_matrix[2,3] ), 'float')
    
    return pos2, quat2, plan_matrix


if __name__ == '__main__':
    param_file_name = "/home/loris/ply_and_stl/param.txt"
    file = open(param_file_name,"r")
    parameters = file.readlines()

    #read parameters from file

    center_x   = taglia_stringa("{(.*?)}",parameters[1],"numero") 
    center_y   = taglia_stringa("{(.*?)}",parameters[2],"numero")
    center_z   = taglia_stringa("{(.*?)}",parameters[3],"numero")
    rho        = taglia_stringa("{(.*?)}",parameters[4],"numero")
    lat_min    = taglia_stringa("{(.*?)}",parameters[5],"numero")
    lat_max    = taglia_stringa("{(.*?)}",parameters[6],"numero")
    long_min    = taglia_stringa("{(.*?)}",parameters[8],"numero")
    long_max    = taglia_stringa("{(.*?)}",parameters[9],"numero")

    rospy.init_node('normalizer', anonymous=True)
    pub1 = rospy.Publisher("o3d_coord_msg", o3d_coord_msg, queue_size=100)
    msg_to_o3d = o3d_coord_msg()
    print('NORMALIZER NODE \n')
    
    
    
    while not rospy.is_shutdown():
        
        print("WAITING FOR THE MESSAGE FROM BAYESIAN OPTIMIZATION NODE")
        bayesian = rospy.wait_for_message("bayesian", bayesian_msg)
        phi_to_be_norm = bayesian.phi
        theta_to_be_norm = bayesian.theta
        
        print("MESSAGE RECEIVED: NORMALIZING COORDINATES")
        phi = normalizer(phi_to_be_norm, lat_min, lat_max)
        phi_rad = phi/180*pi
        theta = normalizer(theta_to_be_norm, long_min, long_max)  
        theta_rad = theta/180*pi
        mat2 = eul2rotm(pi/2,0,-pi/2)
        
        print("CALCULATING COORDINATES")

        sample_points = np.matrix(([rho*np.sin(theta_rad)*np.cos(phi_rad), rho*np.sin(theta_rad)*np.sin(phi_rad),rho*np.cos(theta_rad)]),'float32')
        

        sample_points[0,0] = sample_points[0,0] + center_x
        sample_points[0,1] = sample_points[0,1] + center_y
        sample_points[0,2] = sample_points[0,2] + center_z
        
        rotm_sphere = np.matrix(([np.sin(theta_rad)*np.cos(phi_rad), np.sin(theta_rad)*np.sin(phi_rad), np.cos(theta_rad)],
                         [np.cos(theta_rad)*np.cos(phi_rad), np.cos(theta_rad)*np.sin(phi_rad), -np.sin(theta_rad)],
                         [-np.sin(phi_rad), np.cos(phi_rad), 0]))

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
        
        mat_vet = np.reshape(mat,-1)

        pos = np.matrix(([sample_points[0,0]],
                         [sample_points[0,1]],
                         [sample_points[0,2]]), 'float')

        r = R.from_matrix(mat)
        quat = np.array(r.as_quat(), 'float')

        pos, quat, plan_matrix = fromSENSORtoEEF(pos,quat)
        
        msg_to_o3d.x = pos[0]
        msg_to_o3d.y = pos[1]
        msg_to_o3d.z = pos[2]
        msg_to_o3d.a1 = plan_matrix[0,0]
        msg_to_o3d.a2 = plan_matrix[0,1]
        msg_to_o3d.a3 = plan_matrix[0,2]
        msg_to_o3d.a4 = plan_matrix[1,0]
        msg_to_o3d.a5 = plan_matrix[1,1]
        msg_to_o3d.a6 = plan_matrix[1,2]
        msg_to_o3d.a7 = plan_matrix[2,0]
        msg_to_o3d.a8 = plan_matrix[2,1]
        msg_to_o3d.a9 = plan_matrix[2,2]
    
        rospy.sleep(2)
        pub1.publish(msg_to_o3d)
        print("COORDINATES SENT\n")
        print("-------------------------------------\n")
    
    
