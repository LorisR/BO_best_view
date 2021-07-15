#!/usr/bin/env python3
from stl import mesh
from mpl_toolkits import mplot3d
from matplotlib import pyplot
import math
import numpy as np
from math import pi
import matplotlib.pyplot as plt


def eul2rotm(phi,theta,gamma):
    ''' phi rotazione lungo z, theta rotazione lungo y, gamma rotazione lungo x'''
    phi = np.deg2rad(phi)
    theta = np.deg2rad(theta)
    gamma = np.deg2rad(gamma)
    rz = np.matrix([[np.cos(phi),-np.sin(phi),0],[np.sin(phi),np.cos(phi),0],[0,0,1]])
    ry = np.matrix([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    rx = np.matrix([[1,0,0],[0,np.cos(gamma),-np.sin(gamma)],[0,np.sin(gamma),np.cos(gamma)]])
    rotm = rz*ry*rx
    return rotm    

def rotate_and_tranlate_stl(numpy_vector,phi,theta,gamma,x,y,z):
    ''' as iinput i want variable.vector coming from stl file'''
   
    vector = np.asarray(numpy_vector)
    dim_vector = len(vector)
    vector_r = np.reshape(vector,(dim_vector,9))
    vector_ruotato = np.zeros((dim_vector,9))
    #creo matrice di trasformazione
    #dati di esemèpio
    pose = np.matrix(([x],[y],[z]))
    rot_mat = eul2rotm(phi,theta,gamma)
    hom = np.matrix(([0,0,0, 1]), 'float')
    pos1 = np.hstack((rot_mat, pose)) # attacco la colonna
    pose = np.vstack((pos1, hom)) #attacco la riga
    for i in range (0,dim_vector):
        vett_1 = np.transpose(np.asmatrix(vector_r[i,0:3]))
        vett_2 = np.vstack((vett_1,1))
        result = pose*vett_2
        vector_ruotato[i,0:3] = np.transpose(result[0:3])
        
        vett_1 = np.transpose(np.asmatrix(vector_r[i,3:6]))
        vett_2 = np.vstack((vett_1,1))
        result = pose*vett_2
        vector_ruotato[i,3:6] = np.transpose(result[0:3])
        
        vett_1 = np.transpose(np.asmatrix(vector_r[i,6:9]))
        vett_2 = np.vstack((vett_1,1))
        result = pose*vett_2
        vector_ruotato[i,6:9] = np.transpose(result[0:3])
    
    # ricreo la matrice tridimensionale
    vector = np.reshape(vector_ruotato,(dim_vector,3,3))
    return vector

def sphere_generator(x_centre,y_centre,z_centre,rho,lat_min,lat_max,n_lat,long_min,long_max,n_long):



    lat_min_r = lat_min/180*pi
    lat_max_r = lat_max/180*pi
    long_min_r = long_min/180*pi
    long_max_r = long_max/180*pi
    
    n_df = (lat_max_r-lat_min_r)/n_lat
    m_df = (long_max_r-long_min_r)/n_long
    phi = np.arange(lat_min_r, lat_max_r, n_df)
    theta = np.arange(long_min_r, long_max_r, m_df)

    sample_points = np.zeros((len(phi)*len(theta),3), dtype="float64")
    a=0

    for i in range(0, len(phi)):
        for pp in range(0, len(theta)):
            sample_points[a,0] = rho*np.sin(theta[pp])*np.cos(phi[i])
            sample_points[a,1] = rho*np.sin(theta[pp])*np.sin(phi[i])
            sample_points[a,2] = rho*np.cos(theta[pp])
            
            a = a+1
            
    sample_points[:,0] = sample_points[:,0] +x_centre
    sample_points[:,1] = sample_points[:,1] +y_centre
    sample_points[:,2] = sample_points[:,2] +z_centre
    return sample_points
    

# if __name__ == "__main__":
    
#         # Create a new plot
#     figure = pyplot.figure(0)
#     axes = mplot3d.Axes3D(figure)
#     axes.set_xlim(0,100)
#     axes.set_ylim(0,100)
#     axes.set_zlim(0,100)
#     # Load the STL files and add the vectors to the plot
#     your_mesh = mesh.Mesh.from_file('/home/loris/ply_and_stl/scene_and_model/SEN_MULPOS.stl')    
#     axes.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
#     # Auto scale to the mesh size
#     #scale = your_mesh.points.flatten(-1)
#     #axes.auto_scale_xyz(scale, scale, scale)
#     pyplot.show()
#     vector_r = rotate_and_tranlate_stl(your_mesh.vectors,90,0,0,0,0,0)
#     figure1 = pyplot.figure(1)
#     axes1 = mplot3d.Axes3D(figure1)
#     axes1.set_xlim(0,100)
#     axes1.set_ylim(0,100)
#     axes1.set_zlim(0,100)
#     axes1.add_collection3d(mplot3d.art3d.Poly3DCollection(vector_r))
#     # Auto scale to the mesh size
#     #scale = your_mesh.points.flatten(-1)
#     #axes.auto_scale_xyz(scale, scale, scale)
#     pyplot.show()
# #%%     
#     center_x = 0 #300
#     center_y = 0
#     center_z = 0 #40
#     rho = 200
    
#     n_lat = 10
#     n_long = 10
#     lat_min = 1
#     lat_max =90
#     long_min = 0
#     long_max = 180

#     lat_min_r = lat_min/180*pi
#     lat_max_r = lat_max/180*pi
#     long_min_r = long_min/180*pi
#     long_max_r = long_max/180*pi
    
#     n_df = (lat_max_r-lat_min_r)/n_lat
#     m_df = (long_max_r-long_min_r)/n_long
#     phi = np.arange(lat_min_r, lat_max_r, n_df)
#     theta = np.arange(long_min_r, long_max_r, m_df)

#     sample_points = np.zeros((len(phi)*len(theta),3), dtype="float64")
#     a=0

#     for i in range(0, len(phi)):
#         for z in range(0, len(theta)):
#             sample_points[a,0] = rho*math.cos(phi[i])*math.cos(theta[z])
#             sample_points[a,1] = rho*math.cos(phi[i])*math.sin(theta[z])
#             sample_points[a,2] = rho*math.sin(phi[i])
            
#             a = a+1
        
# #    for b in range(0, len(sample_points)):
#  #      #max_n = max(abs(sample_points[b,0]), abs(sample_points[b,1]), abs(sample_points[b,2]))
#  #       max_n = np.sqrt(np.square(sample_points[b,0])+np.square(sample_points[b,1])+np.square(sample_points[b,2]))
#  #       sample_points[b,3] = sample_points[b,0]/max_n
#  #       sample_points[b,4] = sample_points[b,1]/max_n
#  #       sample_points[b,5] = sample_points[b,2]/max_n

#     rho = 200
#     n_lat = 10
#     n_long = 10
#     lat_min = 1
#     lat_max =90
#     long_min = 0
#     long_max = 180    
    
    
#     sample_points = sphere_generator(0,0,0,rho,lat_min,lat_max,n_lat,long_min,long_max,n_long)    

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')




#     ax.scatter(sample_points[:,0],sample_points[:,1],sample_points[:,2])
    
#     ax.set_xlabel('X Label')
#     ax.set_ylabel('Y Label')
#     ax.set_zlabel('Z Label')
#     print("sono qui")
#     plt.show()    
# #%%    
#     sample_points_vector = np.reshape(sample_points, -1)
    
