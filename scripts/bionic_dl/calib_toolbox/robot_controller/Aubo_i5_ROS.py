import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from std_msgs.msg import String
import rosbag
import numpy as np
import tf
import os
import random
import cv2
import pcl
from calibration import circle_fitting, calibrate
from utils import write_data, read_data
import time
from utils import H2trans_rot
from timeout_decorator import timeout
import multiprocessing as mp
import copy
import sys
from calib_toolbox.utils.transform_ros import pose_from_vector, pose_to_vector

class Aubo_i5_ROS_controller:
    def __init__(self, cfg):
        self.NUM_ATTEMPS = 1
        self.PLANNER_NAME = "RRTConnect"
        self.MAX_PLAN_TIMES = 2
        # initialize moveit_commander and rospy node
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_aubo_i5', anonymous=True)

        # instantiate a RobotCommander object
        self.robot = moveit_commander.RobotCommander()
        # instantiate a Planning SceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()
        # Instantiate a Move Group Commander object
        self.group = moveit_commander.MoveGroupCommander("manipulator_i5")
        # publish trajectory to RVizindex
        self.display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path',
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20)
        # specify planner
        self.group.set_planner_id(self.PLANNER_NAME + 'kConfig1')
        self.group.set_num_planning_attempts(1)
        self.group.set_planning_time(5)

    def get_curr_pose(self):
        pose_ros = self.group.get_current_pose().pose
        return pose_to_vector(pose_ros)

    def move(self, pose_vec):
        pose_ros = pose_from_vector(pose_vec)
        plan_list = []
        self.group.set_start_state_to_current_state()
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose_ros)
        for j in range(self.MAX_PLAN_TIMES):
            plan = self.group.plan()
            if len(plan.joint_trajectory.points) == 0:
                continue
            plan_list.append(plan)
            rospy.sleep(0.5)

        if len(plan_list) == 0:
            print("Fail to get any plan")
            return False

        print("Moving to target:")
        print(pose_vec)
        self.group.execute(plan_list[0])
        rospy.sleep(5)

    def get_joint_pose(self):
        pass
