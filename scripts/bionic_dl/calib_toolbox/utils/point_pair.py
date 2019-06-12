import os
import json
import numpy as np
from .io_old import read_data as read_old
import math
from numpy.linalg import det
import random
import copy
from calib_toolbox.utils.transform import minvec_from_mat, vec_from_mat

class PointPair:
    def __init__(self, index, p_robot=None, p_cam=None):
        self.index = index

        p_robot = np.array(p_robot)
        p_cam = np.array(p_cam)

        if isinstance(p_robot, np.ndarray):
            assert (len(p_robot) == 3), "error: the shape of p_cam is {} (required to be 3x1)".format(p_robot.shape)
            self.p_robot = p_robot
        else:
            self.p_robot = np.ones(3)

        if isinstance(p_cam, np.ndarray):
            assert (len(p_cam) == 3), "error: the shape of p_cam is {} (required to be 3x1)".format(p_cam.shape)
            self.p_cam = p_cam
        else:
            self.p_cam = np.ones(3)

        self.extra_filed = {}

    # FIXME: Fake implementation
    def add_field(self, field, data):
        if field == "joint_pose":
            self.extra_filed.update({"joint_pose": data})
        elif field == "criteria":
            self.extra_filed.update({"criteria": data})

    def get_field(self, field):
        assert (field in self.extra_filed.keys()), "required fied does not exist"
        return self.extra_filed[field]

    def to_ros_msg(self):
        pass

    def to_json(self):
        data_dict = {}
        data_dict.update({"p_robot": self.p_robot.tolist()})
        data_dict.update({"p_cam": self.p_cam.tolist()})
        # TODO: Conversion of information in extra_field
        return data_dict

    def get_mat(self):
        return self.p_robot, self.p_cam


class PointPairList:
    def __init__(self):
        self.pp_dict = {}
        self.used_idx = []
        self.source_dir = None
        self.calib_result = None

    def read_old_data(self, file_path):
        index_list, p_robot_list, p_cam_list = read_old(file_path, "point_pair")
        # pp_dict = {}
        for i in range(len(p_cam_list)):
            self.pp_dict.update({i: PointPair(i, p_robot_list[i], p_cam_list[i])})
        # self.pp_dict = pp_dict
        self.source_dir = file_path

    def read(self, file_path):
        data_dict = {}
        with open(file_path, "r") as f:
            data_dict = json.load(f)

        for i in data_dict.keys():
            p_robot = data_dict[i]["p_robot"]
            p_camera = data_dict[i]["p_cam"]
            i = int(i)
            self.pp_dict.update({i: PointPair(i, p_robot, p_camera)})
        self.source_dir = file_path

    def write_data(self, file_path):
        data_dict = {}
        for idx in self.pp_dict.keys():
            data_dict.update({idx: self.pp_dict[idx].to_json()})
        with open(file_path, "w") as f:
            json.dump(data_dict, f, indent=4)

    def set_result(self):
        pass

    def write_result(self, file_path, H=None, extra_field=None):
        # extra field should include source dir
        if extra_field:
            extra_field.update({"source_data_dir": self.source_dir})
        else:
            extra_field = {"source_data_dir": self.source_dir}

        data_dict = copy.deepcopy(extra_field)

        min_vec = minvec_from_mat(H)
        pose_vec = vec_from_mat(H)
        position = min_vec[:3]
        rot_euler = min_vec[3:]
        rot_quat = pose_vec[3:]

        H = H.flatten()
        data_dict.update({"H": H.tolist()})
        data_dict.update({"position": position.tolist()})
        data_dict.update({"rot_euler_sxyz": rot_euler.tolist()})
        data_dict.update({"rot_quat_wxyz": rot_quat.tolist()})

        output_dir = os.path.join(os.path.dirname(self.source_dir), "calib_result.json")
        with open(output_dir, "w") as f:
            json.dump(data_dict, f, indent=4)
        print("Data output as:{}".format(output_dir))

    def get_len(self):
        return len(self.pp_dict)

    def clear_rand_seeds(self):
        self.used_idx = []

    def valid_test(self, p_robot_mat, p_cam_mat):
        if abs(det(p_cam_mat)) > math.pow(10, -6):
            return True
        else:
            return False

    def _get_mat(self, idx_list):
        if len(idx_list) == 0:
            return None
        else:
            p_robot_mat = []
            p_camera_mat = []
            for idx in idx_list:
                p_robot, p_camera = self.pp_dict[idx].get_mat()
                p_robot_mat.append(p_robot)
                p_camera_mat.append(p_camera)
            p_robot_mat = np.array(p_robot_mat)
            p_robot_mat = np.transpose(p_robot_mat)
            p_robot_mat = np.vstack((p_robot_mat, np.ones(p_robot_mat.shape[1])))

            p_camera_mat = np.array(p_camera_mat)
            p_camera_mat = np.transpose(p_camera_mat)
            p_camera_mat = np.vstack((p_camera_mat, np.ones(p_camera_mat.shape[1])))
            return p_robot_mat, p_camera_mat

    def get_mat(self, num_point=4, idx_list=None):
        if num_point == -1:
            idx_list = list(self.pp_dict.keys())
        elif not idx_list:
            get_result = False
            idx_list = []
            while not get_result:
                while len(idx_list) == 0 or idx_list in self.used_idx:
                    idx_list = random.sample(range(len(self.pp_dict)), num_point)
                self.used_idx.append(idx_list)
                p_robot_mat, p_cam_mat = self._get_mat(idx_list)
                if self.valid_test(p_robot_mat, p_cam_mat):
                    get_result = True

        return self._get_mat(idx_list)

    def get_mat_full(self):
        return self.get_mat(num_point=-1)

    def add_point(self, p_robot, p_camera):
        index = len(self.pp_dict.keys())
        self.pp_dict.update({index: PointPair(index, p_robot, p_camera)})
        return index
