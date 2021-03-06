# Scripts for apply calibration lively
import sys
sys.path.append("..")
from robot_controller.Franka_ROS import Franka_ROS_Controller
from config import cfg
from utils.point_pair import PointPair, PointPairList
from path_generator.path_generator import PathGenerator
from calibration.calib_main import RANSAC, IterativeCalib
from calibration.circle_fitting_main import CircleFitting_V2
from cam_controller.photoneo import Photoneo_Controller
import rospy
import argparse
import os
import datetime
import time
import numpy as np
import copy

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calibration Configs")
    parser.add_argument("--config-file", default="../config_files/online_calib.yaml", metavar="FILE", help="path to config file")
    args = parser.parse_args()

    cfg.merge_from_file(args.config_file)

    rospy.init_node('move_group', anonymous=True)

    robot_controller = Franka_ROS_Controller(cfg)
    camera_controller = Photoneo_Controller(cfg)
    path_generator = PathGenerator(cfg, robot_controller.get_curr_pose())

    # Create folder
    folder_name = time.strftime("%Y-%m-%d", time.localtime(int(round(time.time() * 1000)) / 1000))
    folder_path = os.path.join(cfg.DATA_PATH, folder_name)
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
    pp_list_selected = PointPairList()
    index = -1

    # Start Calibration
    path_generator.curr_idx = 8
    while True:

        # Get next pose from path generator
        pose_next = path_generator.get_waypoint_next()
        if len(pose_next) == 0:
            break
        if index >= len(path_generator.interp_path_list):
            break
        index += 1

        # Move robot
        robot_controller.move(pose_next)

        # Obtain piont cloud from camera controller
        # FIXME: Unreasonable controlling command
        point_cloud_dir = os.path.join(folder_path, "0 "+str(index))
        camera_controller.get_image(point_cloud_dir)
        point_cloud_dir = os.path.join(os.path.dirname(point_cloud_dir), "0"+str(index)+".ply")

        if "Calib" in cfg.MODE:
            # Circle extractor
            circle_extractor = CircleFitting_V2(cfg)
            p_camera = circle_extractor(point_cloud_dir, 0)
        else:
            p_camera = np.array([])

        p_robot = robot_controller.get_curr_pose()

        list_idx = pp_list.add_point(p_robot, p_camera)
        # FIXME: Convert unit from m to mm for temporally use
        pp_list.pp_dict[list_idx].add_field("robot_pose", p_robot)
        pp_list.pp_dict[list_idx].add_field("point_cloud_dir", point_cloud_dir)
        pp_list.write_data(data_path)

        list_idx = pp_list_selected.add_point(p_robot, p_camera)

        if "Calib" in cfg.MODE:
            pp_list.remove_empty()
            if pp_list.get_len() > 4:
                # calib = RANSAC(cfg)
                calib = IterativeCalib(cfg)
                H, index_selected = calib(pp_list)
                pp_list.write_result(result_path, H)
                index_deleted = list(set(pp_list.keys())-set(index_selected))[0]
                pp_list_selected.pp_dict.pop(index_deleted)
