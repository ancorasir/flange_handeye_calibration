from open3d import *
import open3d as o3d

index_ori = 1
data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/10:59:06"
ur5 = read_point_cloud(data_path+"/tool0_%s.pcd"%index_ori)
ur5_circle = read_point_cloud(data_path+"/Tool0_c%s.pcd"%index_ori)

data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/14:45:39"
ur10 = read_point_cloud(data_path+"/tool0_%s.pcd"%index_ori)
ur10_circle = read_point_cloud(data_path+"/Tool0_c%s.pcd"%index_ori)

data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-06-05/16:57:46"
franka = read_point_cloud(data_path+"/tool0_%s.pcd"%index_ori)
franka_ = o3d.io.read_point_cloud(data_path+"/1.ply")
franka_circle = read_point_cloud(data_path+"/Tool0_c%s.pcd"%index_ori)

data_path = "/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid"
aubo = read_point_cloud(data_path+"/tool0_15450226450_530560970306.pcd")
aubo_circle = read_point_cloud(data_path+"/Tool0_c%s.pcd"%46)

o3d.visualization.draw_geometries([ur5])
o3d.visualization.draw_geometries([ur5_circle])

o3d.visualization.draw_geometries([ur10])

o3d.visualization.draw_geometries([franka])
o3d.visualization.draw_geometries([franka_circle])

o3d.visualization.draw_geometries([aubo])
o3d.visualization.draw_geometries([aubo_circle])

ur5_ = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_ur5_cropped.pcd")
ur10_ = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_UR10e_cropped.pcd")
franka_ = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_Franka_cropped.pcd")
aubo_ = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_aubo_cropped.pcd")

o3d.visualization.draw_geometries([ur5_])
o3d.visualization.draw_geometries([ur10_])

o3d.visualization.draw_geometries([franka_])

o3d.visualization.draw_geometries([aubo_])
