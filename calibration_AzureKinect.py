from open3d import *
import open3d as o3d
import numpy as np
import copy, tf
from calibration import get_calibrate

data_path = "./Kinect_Result"
NUM_POINTS = 49

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.transform(transformation)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def evaluate_calibration(s,t,H):
    # sTt = copy.deepcopy(s)
    # sTt.transform(H)
    # HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    # # reg_p2p = registration_icp(sTt, t, 1, HI, TransformationEstimationPointToPoint(),ICPConvergenceCriteria(max_iteration = 200))
    # reg_p2l = registration_icp(
    #         sTt, t, 1, HI,
    #         TransformationEstimationPointToPlane())
    tTs = copy.deepcopy(t)
    tTs = tTs.transform(np.matrix(H).getI())
    HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    reg_p2l = registration_icp(
            s, tTs, 1, HI,
            TransformationEstimationPointToPlane())
    R = reg_p2l.transformation[:3,:3]
    T = reg_p2l.transformation[:3,3]
    al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
    return T, np.array([al, be, ga])/3.14*180


# read point cloud for hand-eye calibration validation
# for ur5, ur10 and franka, we always use the first
target = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_aubo_a.pcd")
target.get_max_bound()
target.get_min_bound()
source = read_point_cloud(data_path+"/flange.pcd") #37
# H_offset = np.matrix([[-1,0,0,31.5],[0,-1,0,31.5],[0,0,1,-6.5],[0,0,0,1]])
H_offset = np.matrix([[-1,0,0,0],[0,1,0,0],[0,0,-1,-6],[0,0,0,1]])
trans = m3d.Transform((-498.88,-261.33,1095.64,0.205,0.811,-2.375))
H_base_tool = trans.get_matrix()
s = copy.deepcopy(source)
t = copy.deepcopy(target)
o3d.estimate_normals(s,search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
o3d.estimate_normals(t, search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1, max_nn=30))
t.transform(H_offset)
t.transform(H_base_tool)

# read calibration points
# data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/10:59:06"
p_camera_mat = np.loadtxt(data_path+"/measureRobot.txt").transpose()
p_robot = np.loadtxt(data_path+"/readRobot.txt").transpose()
p_robot_mat = p_robot[:3,:]
calibrate = get_calibrate(4)
#######################################################
# method 1: calibration using all data points
H = calibrate(p_robot_mat, p_camera_mat)
np.save(data_path+'/H.npy',H)
draw_registration_result(s, t, H)
evaluate_calibration(s,t,H)
tTs = copy.deepcopy(t)
tTs = tTs.transform(np.matrix(H).getI())
HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
reg_p2l = registration_icp(
        s, tTs, 2, HI,
        TransformationEstimationPointToPlane())
HH = reg_p2l.transformation
draw_registration_result(s, tTs, HH)
###################################################################
## test calibration error with photoneo calibration software
import math3d as m3d
target = read_point_cloud("/home/bionicdl/git-projects-py2/flange_handeye_calibration/flange_ground_truth/flange_ur5_cropped.pcd")
target.get_max_bound()
target.get_min_bound()
source = read_point_cloud("./Photoneo_Matrix/tool0_0.pcd") #37
H_offset1_ = np.matrix([[1,0,0,45],[0,0,-1,45],[0,1,0,-6.5],[0,0,0,1]])
H_offset2 = np.matrix([[-1,0,0,0],[0,-1,0,0],[0,0,1,0],[0,0,0,1]])

trans = m3d.Transform((-776.060, -489.340, 1038.320, 0.125, 0.032, 4.016))
H_base_tool = trans.get_matrix()
tf.transformations.euler_from_matrix(H_base_tool[:3,:3])
s = copy.deepcopy(source)
t = copy.deepcopy(target)
o3d.estimate_normals(s,search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1, max_nn=30))
o3d.estimate_normals(t, search_param=o3d.geometry.KDTreeSearchParamHybrid(
        radius=1, max_nn=30))
t.transform(H_offset)
t.transform(H_base_tool)

H=np.loadtxt("./Photoneo_Matrix/16-Matrix(new).txt")
draw_registration_result(s, t, H)

sTt = copy.deepcopy(s)
sTt.transform(H)
HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
reg_p2l = registration_icp(
        sTt, t, 1, HI,
        TransformationEstimationPointToPlane())
HH_ = reg_p2l.transformation
R = reg_p2l.transformation[:3,:3]
T = reg_p2l.transformation[:3,3]
al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
draw_registration_result(sTt, t, HH)
draw_registration_result(s, t, np.matmul(HH,H))

T, np.array([al, be, ga])/3.14*180

tTs = copy.deepcopy(t)
tTs = tTs.transform(np.matrix(H).getI())
reg_p2l = registration_icp(
        s, tTs, 1, HI,
        TransformationEstimationPointToPlane())
HH = reg_p2l.transformation
draw_registration_result(s, tTs, HH)
