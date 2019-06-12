# Scripts for apply calibration lively
import calib_toolbox.calibration as calib
from calib_toolbox.robot_controller.Franka_ROS import Franka_ROS_Controller
from calib_toolbox.calibration.calib_main import make_calibrator
from calib_toolbox.config import cfg
import calib_toolbox.utils as utils
import calib_toolbox.utils.io_old as io_old
from calib_toolbox.utils.point_pair import PointPair, PointPairList
from calib_toolbox.utils import transform as trans
from transforms3d.euler import quat2euler, euler2quat

from calib_toolbox.path_generator.path_generator import PathGenerator
from calib_toolbox.calibration.circle_fitting_main import CircleFitting_V2, CircleFitting_V1
# from calib_toolbox.robot_controller.robot_controller_main import make_robot_controller
# from calib_toolbox.calibration.circle_fitting_main import make_circle_extractor
from calib_toolbox.cam_controller.photoneo import Photoneo_Controller
import rospy
import argparse
import os
import datetime
import time
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibration Configs")
    parser.add_argument("--config-file", default="../configs/SVD.yaml", metavar="FILE", help="path to config file")
    parser.add_argument("--input-dir", default="../data/data_new/test_data.json", metavar="FILE", help="path to data files")
    parser.add_argument("--output-dir", default="/home/bionicdl-Mega/repos/flange_handeye_calibration/scripts/bionic_dl/data/", help="folder to output data files")
    parser.add_argument("--data-type", default="new", help="format of data")
    parser.add_argument("--script-path", default="/home/bionicdl-Mega/repos/flange_handeye_calibration/scripts/bionic_dl/calib_toolbox/utils/Photoneo")
    args = parser.parse_args()


    rospy.init_node('move_group', anonymous=True)
    robot_controller = Franka_ROS_Controller(cfg)
    camera_controller = Photoneo_Controller(args.script_path)
    path_generator = PathGenerator(cfg, robot_controller.get_curr_pose())
    calib = make_calibrator(cfg)
    circle_extractor = CircleFitting_V2()

    script_path = args.script_path
    folder_name = time.strftime("%Y-%m-%d", time.localtime(int(round(time.time() * 1000)) / 1000))
    folder_path = os.path.join(args.output_dir, folder_name)
    folder_path_final = folder_path
    folder_idx = 0

    while os.path.exists(folder_path_final):
        folder_path_final = folder_path + "-" + str(folder_idx)
        folder_idx += 1

    if not os.path.exists(folder_path_final):
        os.makedirs(folder_path_final)
        print("Folder created: %s"%(folder_path_final))

    folder_path = folder_path_final

    data_path = os.path.join(folder_path, "calib_data.json")
    result_path = os.path.join(folder_path, "calib_result.json")
    pp_list = PointPairList()
    index = 0

    while True:

        pose_next = path_generator.get_waypoint_next()
        if len(pose_next) == 0:
            break

        index += 1

        robot_controller.move(pose_next)
        point_cloud_dir = os.path.join(folder_path, "0 "+str(index))
        camera_controller.get_image(point_cloud_dir)
        point_cloud_dir = os.path.join(os.path.dirname(point_cloud_dir), "0"+str(index)+".ply")


        # On-line calibration
        #%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        p_camera = circle_extractor(point_cloud_dir, 0)
        p_robot = robot_controller.get_curr_pose()[:3]
        pp_list.add_point(p_robot, p_camera)
        pp_list.write_data(data_path)

        # Apply Calibration when points are valid
        if pp_list.get_len() > cfg.CALIBRATION.MIN_POINT:
            pp_list.write_data(data_path)
            H = calib(pp_list)
            calib.write_result(result_path, H)
        # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        # # Data collection Only
        # # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        # p_camera = np.array([1, 2, 3])
        # p_robot = robot_controller.get_curr_pose()[:3]
        # pp_list.add_point(p_robot, p_camera)
        # pp_list.write_data(data_path)
        # # %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

