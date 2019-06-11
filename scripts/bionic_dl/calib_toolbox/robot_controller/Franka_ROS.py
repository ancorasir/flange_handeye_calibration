import numpy as np
import rospy
import tf
import sys
import time
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import cv2,os
from calib_toolbox.utils.transform_ros import pose_from_vector, pose_to_vector


class Franka_ROS_Controller:
    def __init__(self, cfg):
        # rospy.init_node()
        moveit_commander.roscpp_initialize(sys.argv)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group = moveit_commander.MoveGroupCommander("panda_arm")
        group.set_planner_id('RRTConnectkConfigDefault')
        group.set_num_planning_attempts(10)
        group.set_planning_time(5)
        group.set_max_velocity_scaling_factor(0.5)
        self.group = group

        tool = PoseStamped()
        tool.header.frame_id = "panda_link0"
        tool.pose.orientation.x = 0
        tool.pose.orientation.y = 0
        tool.pose.orientation.z = 0
        tool.pose.orientation.w = 1
        self.tool = tool

    def move(self, pose_vec):
        group = self.group
        tool  = self.tool

        group.set_start_state_to_current_state()
        group.clear_pose_targets()
        tool.pose = pose_from_vector(pose_vec)
        group.set_pose_target(tool, end_effector_link='panda_link8')
        plan = group.plan()
        res = group.execute(plan)
        time.sleep(2)
        if not res:
            # if fail, swich robot back to mode 2 so that next move can be executed
            # https://github.com/frankaemika/franka_ros/issues/69
            os.system(
                "rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")

    def get_current_pose(self):
        group  = self.group
        pose_ros = self.group.get_current_pose().pose
        return pose_to_vector(pose_ros)

    def get_joint_pose(self):
        pass
