from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, euler2quat, quat2euler, mat2euler, euler2mat
from transforms3d.affines import compose, decompose
import copy
import math
import rospy
import numpy as np


def pose2mat(pos, rot):
    t = np.array(pos)
    r = euler2mat(rot[0], rot[1], rot[2])
    z = np.ones(3)
    pose_mat = compose(t, r, z)
    return np.mat(pose_mat)


def mat2pose(pose_mat):
    t, r, z, s = decompose(pose_mat)
    pos = t
    rot = mat2euler(r)
    return pos, rot


def mat_from_minvec(min_vec):
    """
    Obtain 4x4 matrix from pose in 6D vector
    :param min_vec: [x,y,z,r,p,y]
    :return: 4x4 matrix representing SE3 Transformation
    """
    pos_vec = min_vec[:3]
    euler_vec = min_vec[3:]
    return pose2mat(pos_vec, euler_vec)


def minvec_from_mat(pose_mat):
    """
    Transform 6D vector to SE3 Pose
    :param pose_mat: [x,y,z,r,p,y]
    :return: 4x4 matrix representing SE3 Transformation
    """
    return mat2pose(pose_mat)


def str2float(str_list, data_type="float64"):
    """

    :param str_list: a list of string including numbers
    :param data_type: target data type
    :return: a list of number with required data type
    """
    if data_type:
        return list(np.array(str_list, dtype=data_type))
    else:
        return list(np.array(str_list, dtype="float64"))

