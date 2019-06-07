import os
from yacs.config import CfgNode as CN

_C = CN()

# -----------------------------------------------------------------------------
# Config definition
# -----------------------------------------------------------------------------
_C.DATA = CN()
_C.DATA_PATH = "../../data/data_new"

# -----------------------------------------------------------------------------
# Robot Controller
# -----------------------------------------------------------------------------
_C.ROBOT_CONTROLLER = CN()
# Type of Robot Controller to use
_C.ROBOT_CONTROLLER.ROBOT_TYPE = "Aubo_i5_ROS"
# _C.ROBOT_CONTROLLER.ROBOT_TYPE = "UR5_SOCKET"


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
_C.CALIBRATION.TYPE = "RANSAC"
_C.CALIBRATION.MIN_POINT = 10
# _C.CALIBRATION.TYPE = "SVD"
# -----------------------------------------------------------------------------
# Applicable for RANSAC only
_C.CALIBRATION.PARAM = CN()
_C.CALIBRATION.PARAM.MAX_IT = 10000

# -----------------------------------------------------------------------------
# Path Generation
# -----------------------------------------------------------------------------
_C.PATH_GENERATOR = CN()
# Path for usable point
_C.PATH_GENERATOR.POINT_PATH = "../../data/usable_points.txt"
_C.PATH_GENERATOR.MAX_POINT = -1




