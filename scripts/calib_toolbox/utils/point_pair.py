import os
import json
import numpy as np
from .io_old import read_data as read_old


class PointPair:
    def __init__(self, index, p_robot=None, p_cam=None):
        self.index = index
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

    def to_mat(self):
        pass

    def to_ros_msg(self):
        pass

    def to_json(self):
        data_dict = {}
        data_dict.update({"p_robot": self.p_robot.tolist()})
        data_dict.update({"p_cam": self.p_cam.tolist()})
        # TODO: Conversion of information in extra_field
        return data_dict


class PointPairList:
    def __init__(self):
        self.pp_dict = {}

    def read_old_data(self, data_path):
        index_list, p_robot_list, p_cam_list = read_old(data_path)
        pp_dict = {}
        for i in range(len(p_cam_list)):
            pp_dict.update({i: PointPair(i, p_robot_list[i], p_cam_list[i])})
        self.pp_dict = pp_dict

    def read(self, file_path):
        with open(file_path, "r") as f:
            self.pp_dict = json.load(f)

    def write(self, file_path):
        data_dict = {}
        for idx in self.pp_dict.keys():
            data_dict.update({idx: self.pp_dict[idx].to_json()})
        with open(file_path, "w") as f:
            json.dump(data_dict, f, indent=4)

    def to_mat(self, idx_list):
        pass

    def get_rand(self):
        pass


