import sys
sys.path.append("..")
import os
import math
import numpy as np
from utils.transform import minvec_from_mat, vec_from_mat
import copy
import json
from utils.transform import minvec_from_mat, vec_from_mat, mat_from_vec, mat_from_minvec
from open3d import *
import open3d as o3d
import tf
from numpy.linalg import det

def draw_registration_result(s, t, transformation):
    """
    Visualize transformation result
    :param source: source point cloud
    :param target: target point cloud
    :param transformation: transformation matrix
    :return:
    """
    source_temp = copy.deepcopy(s)
    target_temp = copy.deepcopy(t)
    source_temp.transform(transformation)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    o3d.visualization.draw_geometries([source_temp, target_temp])

def evaluate_calibration(s,t,H):
    """
    Evaluate by ICP
    :param s: source point cloud with reference to camera
    :param t: target point cloud with reference to robot base
    :param H: transformation matrix to be evaluated, camera pose with reference to robot base
    :return:
    (cam',cam)H * cam_P = Base_P * (cam', Base)H
    (cam',cam)H * s = tTs
    """
    tTs = copy.deepcopy(t)
    tTs = tTs.transform(np.matrix(H).getI())
    HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    reg_p2l = o3d.registration_icp(
            s, tTs, 1, HI,
            o3d.TransformationEstimationPointToPlane())
    R = reg_p2l.transformation[:3,:3]
    T = reg_p2l.transformation[:3,3]
    al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
    return T, np.array([al, be, ga])*180/3.1415

def make_calibrator(cfg):
    calib_cfg = cfg.clone()
    if calib_cfg.CALIBRATION.TYPE == "RANSAC":
        print("Using calibrator: {} with MAX_IT:{}".format("RANSAC",calib_cfg.CALIBRATION.PARAM.MAX_IT))
        return RANSAC(cfg)
    elif calib_cfg.CALIBRATION.TYPE == "SVD":
        print("Using calibrator: {}".format("SVD"))
        return SVD(cfg)


class IterativeCalib:
    def __init__(self, cfg):
        # initialize CAD generated point cloud with known offset
        H_offset = np.matrix([[1, 0, 0, -31.5], [0, 1, 0, -31.5], [0, 0, 1, -3.0], [0, 0, 0, 1]])

        target = read_point_cloud(cfg.CALIBRATION.TARGET_DIR)
        source = read_point_cloud(cfg.CALIBRATION.SOURCE_DIR)

        pose_vec_str = cfg.CALIBRATION.SOURCE_POSE
        pose_vec = np.array(pose_vec_str.split(","), dtype=np.float)
        H_base_tool = mat_from_vec(pose_vec)

        s = copy.deepcopy(source)
        t = copy.deepcopy(target)
        o3d.estimate_normals(s,search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=1, max_nn=30))
        o3d.estimate_normals(t, search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=1, max_nn=30))
        t.transform(H_offset)
        t.transform(H_base_tool)
        self.s = s
        self.t = t

    def _cal_SVD(self, p_robot_mat, p_camera_mat):
        """
        Solve for SVD solution
        :param p_robot_mat: 4xi matrix of robot position
        :param p_camera_mat: 4xi matrix of camera position
        :return:
        """
        # p_robot_mat, p_camera_mat = pp_list.get_mat_full()
        num_point = p_robot_mat.shape[1]
        p_robot_centroid = np.mean(p_robot_mat[:3,:],axis=1).reshape(3,1)
        p_camera_centroid = np.mean(p_camera_mat[:3,:],axis=1).reshape(3,1)

        p_robot_demean = p_robot_mat[:3,:] - np.tile(p_robot_centroid,[1,num_point])
        p_camera_demean = p_camera_mat[:3,:] - np.tile(p_camera_centroid,[1,num_point])

        r = np.matrix(np.zeros([3,3]))
        for i in range(num_point):
            r += np.matmul(p_camera_demean[:,i].reshape(3,1),p_robot_demean[:,i].reshape(1,3))

        u, s, vt = np.linalg.svd(r)
        R = np.matmul( vt.transpose(), np.matmul( np.diag([1, 1, np.linalg.det(np.matmul( vt.transpose(),u.transpose()))]), u.transpose() ) )
        t = p_robot_centroid - np.matmul(R, p_camera_centroid)
        return np.array(np.concatenate([np.concatenate([R,t],axis=1),np.array([0, 0, 0, 1]).reshape(1,4)]))

    def __call__(self, *args, **kwargs):
        """
        SVD Calibration
        :param args: PointPairList object
        :param kwargs:
        :return:
        """
        pp_list = args[0]
        p_robot_mat, p_camera_mat = pp_list.get_mat_full()
        if max(p_robot_mat[2,:])<2:
            p_robot_mat = p_robot_mat * 1000
        if max(p_camera_mat[2,:])<2:
            p_camera_mat = p_camera_mat * 1000
        s = self.s
        t = self.t

        p_robot_mat_s = p_robot_mat[:, :4]
        p_camera_mat_s = p_camera_mat[:, :4]
        error_i = 1000
        xyz_i = np.array([1000,1000,1000])
        rpy_i = np.array([1000,1000,1000])
        index_selected = [0,1,2,3]

        for i in range(4, p_robot_mat.shape[1]):
            index_selected_i = index_selected + [i]
            p_robot_mat_i = np.concatenate((p_robot_mat_s, p_robot_mat[:, i].reshape([4, 1])), axis=1)
            p_camera_mat_i = np.concatenate((p_camera_mat_s, p_camera_mat[:, i].reshape([4, 1])), axis=1)
            for j in range(5):
                index_selected_j = copy.copy(index_selected_i)
                p_robot_mat_j = p_robot_mat_i
                p_camera_mat_j = p_camera_mat_i
                p_robot_mat_j = np.delete(p_robot_mat_j, j, 1)
                p_camera_mat_j = np.delete(p_camera_mat_j, j, 1)
                H_j = self._cal_SVD(p_robot_mat_j, p_camera_mat_j)
                xyz_j, rpy_j = evaluate_calibration(self.s, self.t, H_j)
                error_curr = (np.sum(xyz_j ** 2)) ** (0.5)

                if 0 < error_curr < error_i:
                    error_i = (np.sum(xyz_j ** 2)) ** (0.5)
                    H_i = H_j
                    xyz_i = xyz_j
                    rpy_i = rpy_j
                    p_robot_mat_s = p_robot_mat_j
                    p_camera_mat_s = p_camera_mat_j
                    index_selected_j.pop(j)
                    index_selected = index_selected_j
            # if error_i < 0.5:
            #     print("Terminated: calibration error is within 0.5 mm!")
            #     break
            print("**NUM_POINTS:{}  Error:{} mm xyz:{} mm rpy:{}".format(i+1, error_i, xyz_i, rpy_i))
        draw_registration_result(self.s, self.t, H_i)
        return np.array(H_i), index_selected


class RANSAC:
    def __init__(self, cfg):
        cfg_RANSAC = cfg.clone()
        self.max_it = 10000
        self.num_point = 4

    def _get_error(self, H, p_robot_mat, p_camera_mat):
        error_matrix = p_robot_mat - np.matmul(H, p_camera_mat)
        error = np.sum((np.asarray(error_matrix)) ** 2) / p_camera_mat.shape[0] / (p_camera_mat.shape[1] - 4)
        error = np.sqrt(error)
        return error

    def __call__(self, *args, **kwargs):
        pp_list = args[0]
        len_list = pp_list.get_len()
        # Get maximum allowed iteration number
        max_it = int(
            math.factorial(len_list) / (math.factorial(len_list - self.num_point) * math.factorial(self.num_point)) / 3)

        self.max_it = min(self.max_it, max_it)
        p_robot_mat_full, p_camera_mat_full = pp_list.get_mat_full()

        # used_list = []
        min_error = 10010
        H_final = []
        pp_list.clear_rand_seeds()
        for i in range(self.max_it):
            # result = get_rand_list(point_pair_list, num_point=num_point)
            p_mats = pp_list.get_mat(num_point=4)

            if p_mats:
                p_robot_mat, p_camera_mat = p_mats
                p_camera_mat = np.mat(p_camera_mat)
                p_robot_mat = np.mat(p_robot_mat)
                H = np.matmul(p_robot_mat, p_camera_mat.getI())
                error = self._get_error(H, p_robot_mat_full, p_camera_mat_full)
                print("Times:{}  Error:{}".format(i, error))
                if error < min_error:
                    min_error = error
                    H_final = H

        return np.array(H_final)
