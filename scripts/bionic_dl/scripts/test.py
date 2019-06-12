# Script for calibration using existing data

import calib_toolbox.calibration as calib
from calib_toolbox.calibration.calib_main import make_calibrator
from calib_toolbox.config import cfg
import calib_toolbox.utils as utils
import calib_toolbox.utils.io_old as io_old
from calib_toolbox.utils.point_pair import PointPair, PointPairList
from calib_toolbox.utils import transform as trans
from transforms3d.euler import quat2euler, euler2quat
# from calib_toolbox.calibration.calib_algorithms import save_result
import argparse

def read_old_data(data_path):
    """
    Load from old version of data
    :param data_path:
    :return:
    """
    index, p_cam_list, p_robot_list = io_old.read_data(data_path, "point_pair")
    return index, p_robot_list, p_cam_list


def main():
    parser = argparse.ArgumentParser(description="Calibration Configs")
    parser.add_argument("--config-file", default="../configs/SVD.yaml", metavar="FILE", help="path to config file")
    parser.add_argument("--input-dir", default="../data/data_new/test_data.json", metavar="FILE", help="path to data files")
    parser.add_argument("--output-dir", default="../data/data_new/test_result.json", help="folder to output data files")
    parser.add_argument("--data-type", default="new", help="format of data")
    args = parser.parse_args()

    cfg.merge_from_file(args.config_file)

    pp_list = PointPairList()
    print("Input Path:{}. Data type:{}".format(args.input_dir, args.data_type))
    if args.data_type == "old":
        pp_list.read_old_data(args.input_dir)
    else:
        pp_list.read(args.input_dir)

    calib = make_calibrator(cfg)
    H = calib(pp_list)

    # Codes for examine results
    min_vec = trans.minvec_from_mat(H)
    pose_vec = trans.vec_from_mat(H)
    euler_org = min_vec[3:]
    quat_org = pose_vec[3:]
    euler = quat2euler(pose_vec[3:])
    quat = euler2quat(euler_org[0], euler_org[1], euler_org[2])
    H_min = trans.mat_from_minvec(min_vec)
    H_norm = trans.mat_from_vec(pose_vec)
    pp_list.write_result(args.output_dir, H)


if __name__ == "__main__":
    main()
