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

from std_msgs.msg import String
from beginner_tutorials.msg import coord_msg_checked
from planner_online.msg import coord_to_reach
from scipy.spatial.transform import Rotation as R
from planner_online.msg import start_sensor
from planner_online.msg import stop_sensor
from std_msgs.msg import String

#CHANGE ROOT
root = '/home/loris'

def ModelInitialization(modelname):
    
    print('INITIALIZING MODEL')
    
    # cloud_points = cv.ppf_match_3d.loadPLYSimple("%s.ply" % modelname,1)
    # ves = cloud_points    
    pcd = o3d.io.read_point_cloud("%s" % modelname)
    # pc1 = np.asarray(pcd.points, 'float32') 
    # pc2 = np.asarray(pcd.normals, 'float32') 
    downpcd = pcd.voxel_down_sample(voxel_size=0.05) #0.014
    pc1 = np.asarray(downpcd.points, 'float32') 
    pc2 = np.asarray(downpcd.normals, 'float32') 
    pc = np.hstack((pc1,pc2))
    return pc


def ModelTraining (pc):
    
    detector = cv.ppf_match_3d_PPF3DDetector(0.0025, 0.0025)   #0.025, 0.05 standard
    # PRIMO PARAMETRO : abbasso = training + completo
    # SECONDO PARAMETRO : alzo = meno features sono considerate
    print('MODEL TRAINING IN PROGRESS')
    start_time = time.time()
    detector.trainModel(pc)
    end_time = time.time()
    time_training = int(end_time-start_time)
    print('TRAINING COMPLETED in %f secs' %time_training)
    #cv.ppf_match_3d.ppf_match_3d.write()
    return detector


def SurfaceMatching(pc, pcTest, detector):
    
    start_time = time.time()
    N = 2
    print('MATCHING IN PROGRESS')
    results = detector.match(pcTest, 0.003, 0.003) #0.025
    
    print('PERFORMING ICP')
    icp = cv.ppf_match_3d_ICP(300,0.001) #100 standard
    _, results = icp.registerModelToScene(pc, pcTest, results[:N])
    
    print("POSSIBLE POSES: ")
    for i, result in enumerate(results):
        #result.printPose()
        print("\n-- Pose to Model Index %d: NumVotes = %d, Residual = %f\n%s\n" % (result.modelIndex, result.numVotes, result.residual, result.pose))
        if i == 0:   
            pct1 = cv.ppf_match_3d.transformPCPose(pc, result.pose)
            pose1 = result.pose
            residual1 = result.residual
            Votes1 = result.numVotes
        if i == 1:   
            pct2 = cv.ppf_match_3d.transformPCPose(pc, result.pose)
            pose2 = result.pose
            residual2 = result.residual

    residual = np.array((residual1, residual2), 'float32')
    end_time = time.time()
    time_matching = int(end_time-start_time)
    print('MATCHING COMPLETED in %f secs' %time_matching)

    return pose1, pose2, pct1, pct2, Votes1, residual

def SceneNormalsCalculation (scenename, vs):
    
    print('CALCULATING SCENE NORMALS')

    pcd = o3d.io.read_point_cloud("%s" % scenename)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.01, max_nn=80))
    downpcd = pcd.voxel_down_sample(voxel_size=0.01)
    pcTest_d = np.asarray(downpcd.points, 'float32')
    PCNormals = np.asarray(pcd.normals, 'float32')
    PCPoints = np.asarray(pcd.points, 'float32')
    pcTest= np.hstack((PCPoints, PCNormals))
    
    return pcTest, pcTest_d

def DownSamplingandPlot(pct1, pct2, pcTest_d, vs, residual):
    
     pcd = o3d.geometry.PointCloud()
     pcd.points = o3d.utility.Vector3dVector(pct1[:,:3])
     downpcd = pcd.voxel_down_sample(voxel_size=vs)
     pct1_d = np.asarray(downpcd.points, 'float32')
     
     pcd = o3d.geometry.PointCloud()
     pcd.points = o3d.utility.Vector3dVector(pct2[:,:3])
     downpcd = pcd.voxel_down_sample(voxel_size=vs)
     pct2_d = np.asarray(downpcd.points, 'float32')
               
     fig = plt.figure()
     ax = fig.add_subplot(111, projection='3d')
     fig.set_size_inches(10, 8)
     ax.scatter(pcTest_d[:,0], pcTest_d[:,1], pcTest_d[:,2], s=0.5)
     if residual[0]<1:
         ax.scatter(pct1_d[:,0], pct1_d[:,1], pct1_d[:,2], s=1.5, c='r')
     if residual[1]<1:
        ax.scatter(pct2_d[:,0], pct2_d[:,1], pct2_d[:,2], s=0.5, c='g')
     plt.show()
    
     return 

if __name__ == '__main__':
      
        rospy.init_node('Pose_Estimation_Algorithm', anonymous=True)
        messaggio_ricevuto = coord_msg_checked()
        print('POSE ESTIMATOR NODE \n')
        pub = rospy.Publisher('Matching_Finished', String, queue_size=10)
        pub2 = rospy.Publisher('Request_New_Coord', String, queue_size=10)
        
        #file = np.array((0,0,0,0,0), 'float32')
        j=0
        i=0
        try:
            
            # directory = '/home/marco/ws_moveit/src/ai_moveit_mp/doc/surface_matching/samples/data/'
            # directory1 = '/home/marco/ws_moveit/src/ai_moveit_mp/doc/surface_matching/samples/data/'
            # modelname = "parasaurolophus_6700_1"
            # scenename = "rs1_normals_1"
            # modelname = directory+modelname
            # scenename = directory1+scenename
            scenename = root+'/catkin_ws/src/planner_online/ply_sensor/sensor_pc.ply'
            modelname = root+'/ply_and_stl/scene_and_model/L_con_raccordi.ply'
            
            pc = ModelInitialization(modelname)
            detector = ModelTraining(pc)
            vs = 0.01
            ii=1
            
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
                pose = np.vstack((pos1, hom))
                
                print('WAITING THE SAVING OF SENSOR POINTCLOUD') 
                rospy.wait_for_message('sensor_stop', stop_sensor)
                print('MESSAGE RECEIVED')
                rospy.sleep(2)
                
                pcd = o3d.io.read_point_cloud("%s" % scenename)
                scene_points = np.asarray(pcd.points, 'float32') 
                scene_t = cv.ppf_match_3d.transformPCPose(scene_points, pose)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(scene_t)
                o3d.io.write_point_cloud(root+"/catkin_ws/src/planner_online/ply_sensor/sensor_pc_t.ply",pcd)
                scenename = root+'/catkin_ws/src/planner_online/ply_sensor/sensor_pc_t.ply'
                print('TRANSLATED SENSOR POINT CLOUD wrt ORIGIN SAVED') 
                
                # if i==1:
                #     modelname = '/home/marco/ply_and_stl/scene_and_model/L_con_raccordi_half.ply'
                #     pc = ModelInitialization(modelname)
                #     detector = ModelTraining(pc)
                    
                
                pcTest, pcTest_d = SceneNormalsCalculation(scenename, vs)
                
                pose1, pose2, pct1, pct2, Votes1, residual = SurfaceMatching(pc, pcTest, detector)
                
                DownSamplingandPlot(pct1, pct2, pcTest_d, vs, residual)
                
                num = '%d' %ii
                txt = root+'/catkin_ws/src/planner_online/ply_sensor/hom_matrix'+num+'.txt'
                np.savetxt(txt, pose1)
                ii= ii+1
                print('REQUESTED NEW SET OF COORDINATES')
                pub2.publish('REQUESTING NEW SET OF COORDINATES')
                print('--------------------------------------------------------------')

        except rospy.ROSInterruptException():
            pass

 