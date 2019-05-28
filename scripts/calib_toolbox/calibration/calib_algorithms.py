import os
import  math
import numpy as np

def make_calibrator(cfg):
    calib_cfg = cfg.clone()
    if calib_cfg.CALIBRATION.TYPE == "RANSAC":
        return RANSAC(calib_cfg.CALIBRATION.PARAM.MAX_IT)
    elif calib_cfg.CALIBRATION.TYPE == "SVD":
        return SVD()


class SVD:
    def __init__(self):
        pass

    def __call__(self, *args, **kwargs):
        pass


class RANSAC:
    def __init__(self, max_it=10):
        self.max_it = max_it
        self.num_point = 4

    def __call__(self, *args, **kwargs):
        pp_dict = args
        # p_camera_total, p_robot_total = pp2mat(point_pair_list)
        # calibrate = get_calibrate(num_point)

        def error_test(H, p_robot_mat, p_camera_mat):
            error_matrix = p_robot_mat - np.matmul(H, p_camera_mat)
            error = np.sum((np.asarray(error_matrix)) ** 2) / p_camera_mat.shape[0] / (p_camera_mat.shape[1] - 4)
            return error

        len_list = pp_dict.get_len()

        # Get maximum allowed iteration number
        max_it = int(
            math.factorial(len_list) / (math.factorial(len_list - self.num_point) * math.factorial(self.num_point)) / 3)

        self.max_it = max(self.max_it, max_it)

        # used_list = []
        min_error = 10010
        H_final = []
        for i in range(self.max_it):
            # result = get_rand_list(point_pair_list, num_point=num_point)
            p_mats = pp_dict.get_mat_rand()

            if p_mats:
                p_camera_mat, p_robot_mat = p_mats
                H = calibrate(p_robot_mat, p_camera_mat)
                error = error_test(H)
                print("Times:{}  Error:{}".format(i, error))
                if error < min_error:
                    min_error = error
                    H_final = H

