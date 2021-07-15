#!/home/loris/.pyenv/shims/python
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

pcd = o3d.io.read_point_cloud("/home/loris/ply_and_stl/scene_and_model/Scena_Fusoliera.ply")
downpcd = pcd.voxel_down_sample(voxel_size=0.02)
xyz_load = np.asarray(downpcd.points)
print('xyz_load')
print(xyz_load)
print(len(xyz_load))
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')




ax.scatter(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2])
Axes3D.plot(xyz_load[:,0],xyz_load[:,1],xyz_load[:,2],zdir='z')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
print("sono qui")
plt.show()
time.sleep(5)
plt.close('all')
print("sono dopo al close")
