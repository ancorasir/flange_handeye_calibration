from calib_toolbox.robot_controller.Aubo_i5_ROS import Aubo_i5_ROS_controller
from calib_toolbox.robot_controller.UR5_Socket import UR5_Controller_Socket
from calib_toolbox.robot_controller.Franka_ROS import Franka_ROS_Controller

def make_robot_controller(cfg):
    if cfg.ROBOT_CONTROLLER.ROBOT_TYPE == "Aubo_i5_ROS":
        return Aubo_i5_ROS_controller(cfg)
    elif cfg.ROBOT_CONTROLLER.ROBOT_TYPE == "UR5_SOCKET":
        return UR5_Controller_Socket(cfg)
    elif cfg.ROBOT_CONTROLLER.ROBOT_TYPE == "FRANKA_ROS":
        return Franka_ROS_Controller(cfg)
