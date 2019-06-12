# Scripts for apply calibration lively
import calib_toolbox.calibration as calib
from calib_toolbox.calibration.calib_main import make_calibrator
from calib_toolbox.config import cfg
import calib_toolbox.utils as utils
import calib_toolbox.utils.io_old as io_old
from calib_toolbox.utils.point_pair import PointPair, PointPairList
from calib_toolbox.utils import transform as trans
from transforms3d.euler import quat2euler, euler2quat

from calib_toolbox.path_generator.path_generator import PathGenerator
from calib_toolbox.robot_controller.robot_controller_main import make_robot_controller
from calib_toolbox.calibration.circle_fitting_main import make_circle_extractor
import argparse
import os
import datetime
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibration Configs")
    parser.add_argument("--config-file", default="../configs/SVD.yaml", metavar="FILE", help="path to config file")
    parser.add_argument("--input-dir", default="../data/data_new/test_data.json", metavar="FILE", help="path to data files")
    parser.add_argument("--output-dir", default="../data/", help="folder to output data files")
    parser.add_argument("--data-type", default="new", help="format of data")
    args = parser.parse_args()

    robot_controller = make_robot_controller(cfg)
    circle_extractor = make_circle_extractor(cfg)
    calib = make_calibrator(cfg)
    #
    path_generator = PathGenerator(cfg, robot_controller.get_curr_pose())
    # path_generator = PathGenerator(cfg, np.array([1, 2, 3, 4, 5, 6, 7]))

    date_str = str(datetime.datetime.now()).split(" ")[0] + str(datetime.datetime.now()).split(" ")[1]
    folder_path = os.path.join(args.output_dir, date_str)
    print(folder_path)
    os.system("mkdir %s"%(folder_path))

    data_path = os.path.join(folder_path, "data.json")
    result_path = os.path.join(folder_path, "result.json")
    pp_list = PointPairList()

    while True:
        pose_next = path_generator.get_waypoint_next()
        if len(pose_next) < 0:
            break

        robot_controller.move(pose_next)
        point_cloud_path = "../data/"
        p_camera = circle_extractor(point_cloud_path)
        p_robot = robot_controller.get_curr_pose()[:3]
        # new_idx_list.append(pp_list.add_point(p_robot, p_camera))
        pp_list.add_point(p_robot, p_camera)

        # Apply Calibration when points are valid
        if pp_list.get_len() > cfg.CALIBRATION.MIN_POINT:
            pp_list.write_data(data_path)
            H = calib(pp_list)
            calib.write_result(result_path, H)
