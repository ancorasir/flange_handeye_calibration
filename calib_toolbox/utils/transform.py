from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, euler2quat, quat2euler, mat2euler, euler2mat
from transforms3d.affines import compose, decompose
import copy
import math
import numpy as np


def pose2mat(pos, rot_euler, axes="sxyz"):
    """
    Convert vector of position and euler angle to numpy matrix
    :param pos: np.array [x, y, z]
    :param rot_euler: np.array [rot_x, rot_y, rot_z]
    :param axes: corresponding relation between rotation angles and axis
    :return: 4x4 transformation matrix in np.mat
    """
    t = np.array(pos)
    r = euler2mat(rot_euler[0], rot_euler[1], rot_euler[2], axes=axes)
    z = np.ones(3)
    pose_mat = compose(t, r, z)
    return np.mat(pose_mat)


def mat2pose(pose_mat, axes="sxyz"):
    """
    Convert transformation matrix to vector of position and euler angles
    :param pose_mat: 4x4 transformation matrix
    :param axes: rotation axis, default to 'sxyz'
    :return: pose vector : [x, y, z, q_w, q_x, q_y, q_z]
    """
    # if not isinstance(type(pose_mat), np.mat):
    #     pose_mat = np.mat(pose_mat)

    t, r, z, s = decompose(pose_mat)
    pos = t
    rot_euler = mat2euler(r, axes=axes)
    rot_euler = np.array(rot_euler)
    return pos, rot_euler


def mat_from_minvec(min_vec):
    """
    Obtain 4x4 matrix from pose in 6D vector
    :param min_vec: numpy array [x,y,z, r,p,y]
    :return: 4x4 matrix representing SE3 Transformation
    """
    if not isinstance(type(min_vec), np.ndarray):
        min_vec = np.array(min_vec)

    pos_vec = min_vec[:3]
    euler_vec = min_vec[3:]
    return pose2mat(pos_vec, euler_vec)


def minvec_from_mat(pose_mat):
    """
    Transform 6D vector to SE3 Pose
    :param pose_mat: 4x4 transformation matrix in np.mat
    :return: [x, y, z, rot_x, rot_y, rot_z] in np.array
    """
    pos_vec, rot_euler = mat2pose(pose_mat)
    pos_vec = pos_vec.tolist()
    rot_euler = rot_euler.tolist()
    pos_vec.extend(rot_euler)
    return np.array(pos_vec)


def mat_from_vec(pose_vec):
    """
    Convert 7D pose vector (position + quaternion) to transformation matrix
    :param pose_vec: [x, y, z, q_w, q_x, q_y, q_z]
    :return: 4x4 matrix representing SE3 Transformation in np.mat
    """
    if not isinstance(type(pose_vec), np.ndarray):
        pose_vec = np.array(pose_vec)

    pos_vec = pose_vec[:3]
    quat = pose_vec[3:]
    t = np.array(pos_vec)
    r = quat2mat(quat)
    z = np.ones(3)
    pose_mat = compose(t, r, z)
    return np.mat(pose_mat)


def vec_from_mat(pose_mat):
    """
    Convert 4x4 transformation matrix in np.mat to 7D pose vector
    :param pose_mat: 4x4 transformation matrix
    :return: [x, y, z, q_w, q_x, q_y, q_z]
    """
    # if not isinstance(type(pose_mat), np.mat):
    #     pose_mat = np.mat(pose_mat)

    t, r, z, s = decompose(pose_mat)
    pos = t
    quat = mat2quat(r)
    quat = np.array(quat)
    pos = pos.tolist()
    quat = quat.tolist()
    pos.extend(quat)
    return np.array(pos)


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
