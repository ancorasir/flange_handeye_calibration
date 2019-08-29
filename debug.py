import pcl
import numpy as np
from pcl_helper import *
from utils import *
from numpy.linalg import *
data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-06-05/16:57:46"
NUM_POINTS = 80

idx = np.random.choice(p_robot_mat.shape[1],p_robot_mat.shape[1],0)
p_robot_mat_i = p_robot_mat[:,idx[:5]]
p_camera_mat_i = p_camera_mat[:,idx[:5]]
error_i = 1000

i=5
j=0
p_robot_mat_j = p_robot_mat_i
p_camera_mat_j = p_camera_mat_i
p_robot_mat_j = np.delete(p_robot_mat_j, j,1)
p_camera_mat_j = np.delete(p_camera_mat_j, j,1)
H_j = calibrate(p_robot_mat_j, p_camera_mat_j)
xyz_j, rpy_j = evaluate_calibration(s,t,H_j)
xyz_j

draw_registration_result(s, t, H_i)

sTt = copy.deepcopy(s)
sTt.transform(H)
HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
reg_p2l = registration_icp(
        sTt, t, 1, HI,
        TransformationEstimationPointToPlane())
reg_p2p = registration_icp(sTt, t, 1, HI, TransformationEstimationPointToPoint(),ICPConvergenceCriteria(max_iteration = 2000))

reg_p2l.transformation
reg_p2l.fitness
draw_registration_result(sTt, t,reg_p2l.transformation)

error = np.concatenate([np.array(error),data["arr_0"]],axis=0)
xyz = np.concatenate([np.array(xyz),data["arr_1"]],axis=0)
rpy = np.concatenate([np.array(rpy),data["arr_2"]],axis=0)
