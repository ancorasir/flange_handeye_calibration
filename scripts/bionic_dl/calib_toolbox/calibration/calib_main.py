import os
import math
import numpy as np
from calib_toolbox.utils.transform import minvec_from_mat, vec_from_mat
import copy
import json
from calib_toolbox.utils.transform import minvec_from_mat, vec_from_mat
from open3d import *
import tf


def evaluate_calibration(s,t,H):
    sTt = copy.deepcopy(s)
    sTt.transform(H)
    HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    reg_p2p = registration_icp(sTt, t, 0.0003, HI, TransformationEstimationPointToPoint(),ICPConvergenceCriteria(max_iteration = 2000))
    R = reg_p2p.transformation[:3,:3]
    T = reg_p2p.transformation[:3,3]
    al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
    # print("xyz= [%2.2f, %2.2f, %2.2f]"%(T[0]*1000,T[1]*1000,T[2]*1000))
    # print("rpy= [%2.5f, %2.5f, %2.5f]"%(al,be,ga))
    return T*1000, np.array([al, be, ga])/3.14*180


def make_calibrator(cfg):
    calib_cfg = cfg.clone()
    if calib_cfg.CALIBRATION.TYPE == "RANSAC":
        print("Using calibrator: {} with MAX_IT:{}".format("RANSAC",calib_cfg.CALIBRATION.PARAM.MAX_IT))
        return RANSAC(cfg)
    elif calib_cfg.CALIBRATION.TYPE == "SVD":
        print("Using calibrator: {}".format("SVD"))
        return SVD(cfg)


class SVD:
    def __init__(self, cfg):
        # initialize source & target piont cloud
        target = read_point_cloud("/home/bionicdl/photoneo_data/20181217/aubo-i5-EndFlange_cropped_m.pcd")
        source = read_point_cloud("/home/bionicdl/photoneo_data/20181217/tool0_5.pcd")
        H_offset = np.matrix([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, -0.006], [0, 0, 0, 1]])
        H_base_tool = tf.transformations.quaternion_matrix(
            [-0.08580920098798522, -0.3893105864494028, 0.9148593368686363, 0.06408152657751885])
        H_base_tool[:3, 3] = np.array([0.27704067625331485, -0.573055166920657, 0.26205388882758757])
        s = copy.deepcopy(source)
        t = copy.deepcopy(target)
        t.transform(H_offset)
        t.transform(H_base_tool)
        pass

    def __call__(self, *args, **kwargs):
        pp_list = args[0]
        p_robot_mat, p_camera_mat = pp_list.get_mat_full()
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


class RANSAC:
    def __init__(self, cfg):
        cfg_RANSAC = cfg.clone()
        self.max_it = cfg_RANSAC.CALIBRATION.PARAM.MAX_IT
        self.num_point = 4

    @staticmethod
    def _get_error(self, H, p_robot_mat, p_camera_mat):
        error_matrix = p_robot_mat - np.matmul(H, p_camera_mat)
        error = np.sum((np.asarray(error_matrix)) ** 2) / p_camera_mat.shape[0] / (p_camera_mat.shape[1] - 4)
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

