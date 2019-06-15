import os
import json
import numpy as np
from .io_old import read_data as read_old
import math
from numpy.linalg import det
import random
from transform import minvec_from_mat, vec_from_mat


class PointPair:
    def __init__(self, index, p_robot=None, p_cam=None):
        self.index = index

        p_robot = np.array(p_robot)
        p_cam = np.array(p_cam)

        if isinstance(p_robot, np.ndarray):
            self.p_robot = p_robot[:3]
        else:
            self.p_robot = np.array([])

        if isinstance(p_cam, np.ndarray):
            # assert (len(p_cam) == 3), "error: the shape of p_cam is {} (required to be 3x1)".format(p_cam.shape)
            self.p_cam = p_cam[:3]
        else:
            self.p_cam = np.array([])

        self.extra_filed = {}

    def add_field(self, field, data):
        """
        Add additional information for point pair as entity in dictionary
        :param field: name of the field
        :param data: data to be stored
        :return:
        """
        self.extra_filed.update({field: data})

    def get_field(self, field):
        """
        Get data from extra field
        :param field: name of the field
        :return:
        """
        assert (field in self.extra_filed.keys()), "required field does not exist"
        return self.extra_filed[field]

    def to_json(self):
        """
        Covert information in point pair to string of json format
        :return:
        """

        data_dict = {}
        data_dict.update({"p_robot": self.p_robot.tolist()})
        data_dict.update({"p_cam": self.p_cam.tolist()})
        for key in self.extra_filed.keys():
            if isinstance(self.extra_filed[key], np.ndarray):
                self.extra_filed[key] = self.extra_filed[key].tolist()
            data_dict.update({key: self.extra_filed[key]})
        return data_dict

    def get_mat(self):
        """
        Get matrix of current point pair
        :return:
        """
        return self.p_robot, self.p_cam


class PointPairList:
    def __init__(self):
        self.pp_dict = {}
        self.used_idx = []
        self.source_dir = None
        self.calib_result = None

    def read_old_data(self, file_path):
        """
        Read from old format of data
        :param file_path:
        :return:
        """
        index_list, p_robot_list, p_cam_list = read_old(file_path, "point_pair")
        # pp_dict = {}
        for i in range(len(p_cam_list)):
            self.pp_dict.update({i: PointPair(i, p_robot_list[i], p_cam_list[i])})
        # self.pp_dict = pp_dict
        self.source_dir = file_path

    def read(self, file_path):
        """
        Read information from json file
        :param file_path: path of the json file
        :return:
        """

        data_dict = {}
        curr_pp_dict = {}
        with open(file_path, "r") as f:
            data_dict = json.load(f)

        print(data_dict.keys())
        for i in data_dict.keys():
            i = int(i)
            data_dict.update({i:data_dict.pop(str(i))})
            p_robot = np.array(data_dict[i]["p_robot"])
            p_camera = np.array(data_dict[i]["p_cam"])
        
            curr_pp_dict.update({i: PointPair(i, p_robot, p_camera)})

            if len(list(data_dict[i].keys())) > 2:
                for key in data_dict[i].keys():
                    key = str(key)
                    if key != "p_robot" and key !="p_cam":
                        curr_pp_dict[i].add_field(key, data_dict[i][key])

        self.pp_dict.update(curr_pp_dict)
        self.source_dir = file_path

    def write_data(self, file_path):
        """
        Write data to json file
        :param file_path:
        :return:
        """
        data_dict = {}
        for idx in self.pp_dict.keys():
            data_dict.update({idx: self.pp_dict[idx].to_json()})
        with open(file_path, "w") as f:
            json.dump(data_dict, f, indent=4)


    def write_result(self, file_path, H=None):
        """
        Write result to json file
        :param file_path:
        :param H:
        :return:
        """

        data_dict = {}
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
        """
        Get total length of current dictionary
        :return:
        """

        return len(self.pp_dict)

    def clear_rand_seeds(self):
        """
        Reset random seed for obtain random matrix
        :return:
        """
        self.used_idx = []

    def valid_test(self, p_robot_mat, p_cam_mat):
        """
        Test the determinant of matrix
        :param p_robot_mat:
        :param p_cam_mat:
        :return:
        """
        if abs(det(p_cam_mat)) < math.pow(10, -4):
            return False
        else:
            return True

    def _get_mat(self, idx_list):
        """
        Get matrix with given index list. private method within the class
        :param idx_list:
        :return:
        """
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

            return None, None


    def get_mat(self, num_point=4, idx_list=[]):
        """
        Get matrix with required number of points of index list
        :param num_point:
        :param idx_list:
        :return:
        """
        if num_point == -1:
            idx_list = list(self.pp_dict.keys())
        elif len(idx_list) == 0:
            get_result = False
            idx_list = []
            while not get_result:
                while len(idx_list) == 0 or idx_list in self.used_idx:

                    key_list = list(self.pp_dict.keys())
                    # idx_list = random.sample(range(len(self.pp_dict)), num_point)
                    key_idx_list = random.sample(range(len(key_list)), num_point)
                    idx_list = []
                    for i in key_idx_list:
                        idx_list.append(key_list[i])

                self.used_idx.append(idx_list)
                p_robot_mat, p_cam_mat = self._get_mat(idx_list)
                if self.valid_test(p_robot_mat, p_cam_mat):
                    get_result = True
        return self._get_mat(idx_list)

    def get_mat_full(self):
        """
        Obtain p_robot_mat and p_camera_mat (in homogenous coordinate) made up of all point pairs
        :return: p_robot_mat, p_camera_mat
        """
        return self.get_mat(num_point=-1)

    def add_point(self, p_robot, p_camera):
        index = len(self.pp_dict.keys())
        self.pp_dict.update({index: PointPair(index, p_robot, p_camera)})
        return index

    def circle_extraction(self, circle_extractor,overwrite=False, update_all=False):
        """
        Extract circle center for data in current piont pair list
        :param circle_extractor: Circle extractor object
        :param overwrite: Whether to overwrite p_cam on oiginal data
        :param update_all: Whether to apply circle extractor to all data or only to those without p_cam
        :return:
        """

        for i in self.pp_dict.keys():
            point_pair = self.pp_dict[i]

            if len(point_pair.p_cam) == 0 or update_all:
                print("Circle updating for:%s"%(str(i)))
                point_cloud_dir = os.path.join(os.path.dirname(self.source_dir), "0"+str(i)+".ply")
                p_cam = circle_extractor(point_cloud_dir)
                point_pair.p_cam = p_cam

            if len(point_pair.p_cam) != 0:
                print("Circle updated")
            else:
                print("Fail")
            print("-----------------------------")

            if overwrite:
                self.write_data(self.source_dir)
            else:
                self.write_data(os.path.joint(os.path.dirname(self.source_dir), "calib_data_updated.json"))

    def remove_empty(self):
        """
        Remove data entities without p_cam
        :return:
        """
        for i in self.pp_dict.keys():
            point_pair = self.pp_dict[i]
            if len(point_pair.p_cam) == 0:
                self.pp_dict.pop(i)
