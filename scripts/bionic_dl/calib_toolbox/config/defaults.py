import os
from yacs.config import CfgNode as CN

_C = CN()
# Mode:
#     Collect: Collect data
#     Calib: Circle Extraction & Calibration
#     Collect+Calib: On-line calibration
_C.MODE = "Collect"
# -----------------------------------------------------------------------------
# Config definition
# -----------------------------------------------------------------------------
_C.DATA = CN()
_C.DATA_PATH = "/home/bionicdl-Mega/repos/flange_handeye_calibration/scripts/bionic_dl/data/"

# -----------------------------------------------------------------------------
# Robot Controller
# -----------------------------------------------------------------------------
_C.ROBOT_CONTROLLER = CN()
# Type of Robot Controller to use
_C.ROBOT_CONTROLLER.ROBOT_TYPE = "Franka_ROS"
# _C.ROBOT_CONTROLLER.ROBOT_TYPE = "Aubo_i5_ROS"
# _C.ROBOT_CONTROLLER.ROBOT_TYPE = "UR5_SOCKET"
# -----------------------------------------------------------------------------
_C.ROBOT_CONTROLLER.PARAM = CN()
_C.ROBOT_CONTROLLER.PARAM.IP = "192.168.1.27"

# -----------------------------------------------------------------------------
# Camera Controller
# -----------------------------------------------------------------------------
_C.CAM_CONTROLLER = CN()
_C.CAM_CONTROLLER.TYPE = "Photoneo"
_C.CAM_CONTROLLER.SCRIPT_PATH = "/home/bionicdl-Mega/repos/flange_handeye_calibration/scripts/bionic_dl/calib_toolbox/utils/Photoneo"

# -----------------------------------------------------------------------------
# Circle Fitting
# -----------------------------------------------------------------------------
_C.CIRCLE_FITTING = CN()
# Type of circle fitting algorithm to use
_C.CIRCLE_FITTING.TYPE = "V2"

# -----------------------------------------------------------------------------
# Calibration
# -----------------------------------------------------------------------------
_C.CALIBRATION = CN()
# Calibration algorithm, ["RANSAC", "DIRECT", "SVD"]
# _C.CALIBRATION.TYPE = "RANSAC"
_C.CALIBRATION.MIN_POINT = 5
_C.CALIBRATION.TYPE = "SVD"
_C.CALIBRATION.TARGET_DIR = "/home/bionicdl-Mega/repos/new/flange_handeye_calibration/flange_ground_truth/flange_franka.pcd"
_C.CALIBRATION.SOURCE_DIR = "/home/bionicdl-Mega/repos/new/flange_handeye_calibration/flange_ground_truth/flange_franka_ref.pcd"
_C.CALIBRATION.SOURCE_POSE = "494.6041199503259, 114.02491027266817, 455.9592421126155, 0.9144343155379376, -0.01431055883584167, -0.0009572324991887879, 0.40448012828618896"
# _C.CALIBRATION.SOURCE_DIR = "/home/bionicdl-Mega/repos/new/flange_handeye_calibration/flange_ground_truth/franka_ref2.pcd"
# _C.CALIBRATION.SOURCE_POSE = "294.8924164926116, 23.64333038073283, 547.8668618866172, 0.891711695746443, -0.024948206094703076, -0.11366582231497009, -0.4373876078710059"

# -----------------------------------------------------------------------------
# Path Generation
# -----------------------------------------------------------------------------
_C.PATH_GENERATOR = CN()
# Path for piont file
_C.PATH_GENERATOR.POINT_PATH = "../../configs/valid_pos.txt"




