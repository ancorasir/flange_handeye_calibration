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
# Whether or not to use ROS
_C.ROBOT_CONTROLLER.USE_ROS = True
# Type of Robot Controller to use
_C.ROBOT_CONTROLLER.ROBOT_TYPE = "Aubo_i5"


# -----------------------------------------------------------------------------
# Circle Fitting
# -----------------------------------------------------------------------------
_C.CIRCLE_FITTING = CN()
# Type of circle fitting algorithm to use
_C.CIRCLE_FITTING.TYPE = "latest"


# -----------------------------------------------------------------------------
# Calibration
# -----------------------------------------------------------------------------
_C.CALIBRATION = CN()
# Calibration algorithm, ["RANSAC", "DIRECT", "SVD"]
_C.CALIBRATION.TYPE = "DIRCT"
# -----------------------------------------------------------------------------
# Applicable for RANSAC only
_C.CALIBRATION.PARAM.MAX_IT = 10
