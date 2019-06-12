import urx
from calib_toolbox.utils.transform import  vec_from_mat, minvec_from_mat

class UR5_Controller_Socket:
    def __init__(self, cfg):
        self.robot_ip = cfg.ROBOT_CONTROLLER.PARAM.IP
        self.robot = urx.Robot(self.robot_ip)

    def get_curr_pose(self):
        return vec_from_mat(self.robot.get_pose())

    def move(self, pose_vec):
        pass
