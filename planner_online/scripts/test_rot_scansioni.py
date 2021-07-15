import open3d as o3d
import  numpy as np

dir_scan = "/home/loris/prove_halcon/"
pcd_1 = o3d.io.read_point_cloud(dir_scan +"/cassettiera_sx.ply")
pcd_2 = o3d.io.read_point_cloud(dir_scan +"/cassettiera_sopra.ply")
pcd_3 = o3d.io.read_point_cloud(dir_scan +"/cassettiera_dx.ply")
pcd_4 = o3d.io.read_point_cloud(dir_scan +"/total_robot_p2.ply")
# pcd_5 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_5.ply")
# pcd_6 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_6.ply")
# pcd_7 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_7.ply")
# pcd_8 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_8.ply")
# pcd_9 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_9.ply")
# pcd_10 = o3d.io.read_point_cloud(dir_scan +"/ply_scan_10.ply")



pcd_1.paint_uniform_color([.7, 0.7, .7])
pcd_2.paint_uniform_color([.7, 0.7, .7])
pcd_3.paint_uniform_color([.7, 0.7, .7])
pcd_4.paint_uniform_color([.7, 0.7, .7])
# pcd_5.paint_uniform_color([.7, 0.7, .7])
# pcd_6.paint_uniform_color([.7, 0.7, .7])
# pcd_7.paint_uniform_color([.7, 0.7, .7])
# pcd_8.paint_uniform_color([.7, 0.7, .7])
# pcd_9.paint_uniform_color([.7, 0.7, .7])
# pcd_10.paint_uniform_color([.7, 0.7, .7])

matrix_name = "/home/loris/prove_halcon/mat_comp_model.txt"


filex = (np.loadtxt(matrix_name, 'float32'))
t1 = filex[0:4,0:4]
t2 = filex[4:8,0:4]
t3 = filex[8:12,0:4]
t4 = filex[12:16,0:4]
# t5 = filex[16:20,0:4]
# t6 = filex[20:24,0:4]
# t7 = filex[24:28,0:4]
# t8 = filex[28:32,0:4]
# t9 = filex[32:36,0:4]
# t10 = filex[36:40,0:4]

t2_inv = np.linalg.inv(t2)
t3_inv = np.linalg.inv(t3)
t4_inv = np.linalg.inv(t4)
# t5_inv = np.linalg.inv(t5)
# t6_inv = np.linalg.inv(t6)
# t7_inv = np.linalg.inv(t7)
# t8_inv = np.linalg.inv(t8)
# t9_inv = np.linalg.inv(t9)
# t10_inv = np.linalg.inv(t10)

tt1 = np.matmul(t1,t2_inv)
tt2 = np.matmul(t1,t3_inv)
tt3 = np.matmul(t1,t4_inv)
# tt4 = np.matmul(t1,t5_inv)
# tt5 = np.matmul(t1,t6_inv)
# tt6 = np.matmul(t1,t7_inv)
# tt7 = np.matmul(t1,t8_inv)
# tt8 = np.matmul(t1,t9_inv)
# tt9 = np.matmul(t1,t10_inv)


pcd_2.transform(tt1)
pcd_3.transform(tt2)
pcd_4.transform(tt3)
# pcd_5.transform(tt4)
# pcd_6.transform(tt5)
# pcd_7.transform(tt6)
# pcd_8.transform(tt4)
# pcd_9.transform(tt5)
# pcd_10.transform(tt6)
#%% calcolato tutto
pcd_tot = pcd_1+pcd_2+pcd_3+pcd_4
# pcd_tot = pcd_1+pcd_2+pcd_3+pcd_4+pcd_5+pcd_6+pcd_7

downpcd = pcd_tot.voxel_down_sample(voxel_size=0.002)
downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(downpcd)
vis.get_render_option().load_from_json("/home/loris/catkin_ws/src/result_viewer/viewer_settings/viewer.json")
vis.run()
vis.destroy_window()


#o3d.visualization.draw_geometries([downpcd])
# o3d.io.write_point_cloud("/home/loris/prove_halcon/total_robot.ply",downpcd)