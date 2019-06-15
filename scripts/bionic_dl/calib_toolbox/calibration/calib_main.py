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
import tf
from numpy.linalg import det


def evaluate_calibration(s,t,H):
    """
    Evaluate by ICP
    :param s: source point cloud
    :param t: target point cloud
    :param H: transformation matrix to be evaluated
    :return:
    """
    sTt = copy.deepcopy(s)
    sTt.transform(H)
    HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    reg_p2p = registration_icp(sTt, t, 0.0003, HI, TransformationEstimationPointToPoint(),ICPConvergenceCriteria(max_iteration = 200000))
    R = reg_p2p.transformation[:3,:3]
    T = reg_p2p.transformation[:3,3]
    al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
    # print("xyz= [%2.2f, %2.2f, %2.2f]"%(T[0]*1000,T[1]*1000,T[2]*1000))
    # print("rpy= [%2.5f, %2.5f, %2.5f]"%(al,be,ga))
    return T, np.array([al, be, ga])/3.14*180


def draw_registration_result(source, target, transformation):
    """
    Visualize transformation result
    :param source: source point cloud
    :param target: target point cloud
    :param transformation: transformation matrix
    :return:
    """
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])


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
        H_offset = np.matrix([[1, 0, 0, -315], [0, 1, 0, -315], [0, 0, 1, -30], [0, 0, 0, 1]])

        target = read_point_cloud(cfg.CALIBRATION.TARGET_DIR)
        source = read_point_cloud(cfg.CALIBRATION.SOURCE_DIR)

        pose_vec_str = cfg.CALIBRATION.SOURCE_POSE
        pose_vec = np.array(pose_vec_str.split(","), dtype=np.float)
        H_base_tool = mat_from_vec(pose_vec)

        s = copy.deepcopy(source)
        t = copy.deepcopy(target)

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
        error = []
        xyz = []
        rpy = []

        p_robot_mat, p_camera_mat = pp_list.get_mat_full()
        s = self.s
        t = self.t

        # iterate for 100 times
        for r in range(100):
            error_r = []
            xyz_r = []
            rpy_r = []
            init_success = False
            while not init_success:
                idx = np.random.choice(p_robot_mat.shape[1], p_robot_mat.shape[1], 0) # randomly arrange
                p_robot_mat_i = p_robot_mat[:, idx[:5]]
                p_camera_mat_i = p_camera_mat[:, idx[:5]]
                for j in range(5):
                    p_robot_mat_j = p_robot_mat_i
                    p_camera_mat_j = p_camera_mat_i
                    p_robot_mat_j = np.delete(p_robot_mat_j, j, 1)
                    p_camera_mat_j = np.delete(p_camera_mat_j, j, 1)
                    if abs(det(p_robot_mat_j)) > math.pow(10, -4) and abs(det(p_camera_mat_j)) > math.pow(10, -4):
                        init_success = True

            error_i = 1000
            for i in range(5, len(idx)):
                for j in range(5):
                    p_robot_mat_j = p_robot_mat_i
                    p_camera_mat_j = p_camera_mat_i
                    p_robot_mat_j = np.delete(p_robot_mat_j, j, 1)
                    p_camera_mat_j = np.delete(p_camera_mat_j, j, 1)
                    H_j = self._cal_SVD(p_robot_mat_j, p_camera_mat_j)

                    draw_registration_result(self.s, self.t, H_j)
                    xyz_j, rpy_j = evaluate_calibration(self.s, self.t, H_j)

                    error_curr = (np.sum(xyz_j ** 2)) ** (0.5)

                    if 0 < error_curr < error_i:
                        error_i = (np.sum(xyz_j ** 2)) ** (0.5)
                        H_i = H_j
                        xyz_i = xyz_j
                        rpy_i = rpy_j
                        p_robot_mat_s = p_robot_mat_j
                        p_camera_mat_s = p_camera_mat_j
                # print("Iteration:{}  Error:{} mm xyz:{} mm rpy:{}".format(i, error_i, xyz_i, rpy_i))
                error_r.append(error_i)
                xyz_r.append(xyz_i)
                rpy_r.append(rpy_i)
                # if error_i < 0.5:
                #     print("Terminated: calibration error is within 0.5 mm!")
                #     break
                p_robot_mat_i = np.concatenate((p_robot_mat_s, p_robot_mat[:, idx[i]].reshape([3, 1])), axis=1)
                p_camera_mat_i = np.concatenate((p_camera_mat_s, p_camera_mat[:, idx[i]].reshape([3, 1])), axis=1)
            print("Random:{}  Error:{} mm xyz:{} mm rpy:{}".format(r, error_i, xyz_i, rpy_i))
            error.append(error_r)
            xyz.append(xyz_r)
            rpy.append(rpy_r)
            # TODO: Keep the results
        return np.array(H_i)



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

