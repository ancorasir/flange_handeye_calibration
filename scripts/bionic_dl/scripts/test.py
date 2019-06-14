# Script for calibration using existing data
from calib_toolbox.config import cfg
import calib_toolbox.utils.io_old as io_old
from calib_toolbox.utils.point_pair import PointPair, PointPairList
from calib_toolbox.calibration.calib_main import SVD, RANSAC
from calib_toolbox.calibration.circle_fitting_main import CircleFitting_V2, CircleFitting_V1
import os
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
    parser.add_argument("--config-file", default="/home/bionicdl-Mega/repos/flange_handeye_calibration/scripts/bionic_dl/configs/offline_calib.yaml", metavar="FILE", help="path to config file")
    parser.add_argument("--data-type", default="new", help="format of data")
    args = parser.parse_args()

    cfg.merge_from_file(args.config_file)

    data_path = cfg.DATA_PATH
    result_path = os.path.join(os.path.dirname(data_path), "calib_result.json")

    pp_list = PointPairList()
    print("Input Path:{}. Data type:{}".format(data_path, args.data_type))
    if args.data_type == "old":
        pp_list.read_old_data(data_path)
    else:
        pp_list.read(data_path)

    calib = SVD(cfg)
    circle_extractor = CircleFitting_V2(cfg)

    # pp_list.circle_extraction(circle_extractor, overwrite=True, update_all=False)
    pp_list.remove_empty()

    H = calib(pp_list)
    pp_list.write_result(args.output_dir, H)


if __name__ == "__main__":
    main()
