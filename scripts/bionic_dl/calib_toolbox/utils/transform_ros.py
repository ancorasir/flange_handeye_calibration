from transforms3d.quaternions import quat2mat, mat2quat
from transforms3d.euler import euler2mat, euler2quat, quat2euler, mat2euler, euler2mat
from transforms3d.affines import compose, decompose
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion
import tf.transformations as trans
from std_msgs.msg import String
import numpy as np


def point_to_vector(point_msg):
    return [point_msg.x, point_msg.y, point_msg.z]


def point_from_vector(point_vec):
    point_msg = Point()
    point_msg.x = point_vec[0]
    point_msg.y = point_vec[1]
    point_msg.z = point_vec[2]
    return point_msg


def quat_to_vector(quat_msg):
    """ geometry_msgs/Quaternion to vector [w, x, y, z] """
    return [quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z]


def quat_from_vector(quat_vec):
    """ vector [w, x, y, z] to geometry_msgs/Quaternion """
    quat_msg = Quaternion()
    quat_msg.w = quat_vec[0]
    quat_msg.x = quat_vec[1]
    quat_msg.y = quat_vec[2]
    quat_msg.z = quat_vec[3]
    return quat_msg


def quat_to_euler_vec(quat_msg):
    """ geometry_msgs/Quaternion to vector [r, p, y] """
    return quat2euler(quat_to_vector(quat_msg))


def quat_from_euler_vec(euler_vec):
    """ geometry_msgs/Quaternion from vector [r, p, y] """
    return quat_from_vector(
        euler2quat(euler_vec[0], euler_vec[1], euler_vec[2]))


def pose_to_minVector(pose_msg):
    """ geometry_msgs/Pose to vector [x, y, z, roll, pitch, yaw] """
    pos_vec = point_to_vector(pose_msg.position)
    euler_vec = quat_to_euler_vec(pose_msg.orientation)
    pos_vec.extend(euler_vec)
    return np.array(pos_vec)


def pose_from_minVector(pose_minvec):
    """ geometry_msgs/Pose from vector [x, y, z, roll, pitch, yaw] """
    pos_vec = pose_minvec[:3]
    euler_vec = pose_minvec[3:]
    pose_msg = Pose()
    pose_msg.position = point_from_vector(pos_vec)
    pose_msg.orientation = quat_from_euler_vec(euler_vec)
    return pose_msg


def pose_to_vector(pose_msg):
    """ geometry_msgs/Pose to vector [x, y, z, quat_w, quat_x, quat_y, quat_z] """
    pos_vec = point_to_vector(pose_msg.position)
    quat_vec = quat_to_vector(pose_msg.orientation)
    pos_vec.extend(quat_vec)
    return np.array(pos_vec)


def pose_from_vector(pose_vec):
    """ geometry_msgs/Pose from vector [x, y, z, quat_w, quat_x, quat_y, quat_z] """
    pos_vec = pose_vec[:3]
    quat_vec = pose_vec[3:]
    pose_msg = Pose()
    pose_msg.position = point_from_vector(pos_vec)
    pose_msg.orientation = quat_from_vector(quat_vec)
    return pose_msg


def print_pose_msg(title, pose_msg, angle_type="euler"):
    msg = ""
    pose_vec = []
    if angle_type == "quat":
        pose_vec = pose_to_vector(pose_msg)
    elif angle_type == "euler":
        pose_vec = pose_to_minVector(pose_msg)
    print(str(title) + " :" + str(pose_vec))

