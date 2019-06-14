import rospy
import time
import numpy as np

from calib_toolbox.path_generator.path_generator import PathGenerator
from calib_toolbox.config import cfg

if __name__ == "__main__":
    # test for franka robot
    rospy.init_node('move_group', anonymous=True)
    from calib_toolbox.robot_controller.Franka_ROS import Franka_ROS_Controller
    robot_controller = Franka_ROS_Controller(None)
    curr_pose = robot_controller.get_curr_pose()
    print(curr_pose.tolist())
    # path_generator = PathGenerator(cfg, np.array(curr_pose))
    # while(True):
    #     pose_next = path_generator.get_waypoint_next()
    #     if len(pose_next) == 0:
    #         break
    #     print(pose_next[:3])
    #     robot_controller.move(pose_next)
    #     # time.sleep(3)

    # print(curr_pose)
    # curr_pose[0] += 0.02
    # print(curr_pose)
    # robot_controller.move(curr_pose)
    # time.sleep(5)
