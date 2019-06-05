#!/usr/bin/env python
import numpy as np
import rospy, tf, sys, time
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import tf2_ros
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
import cv2,os

timestamp=time.strftime('data/%Y-%m-%d/%H:%M:%S',time.localtime(int(round(time.time()*1000))/1000))
folder=time.strftime('data/%Y-%m-%d',time.localtime(int(round(time.time()*1000))/1000))
if not os.path.exists(timestamp):
    os.makedirs(timestamp)

# TODO: Construct 3D calibration grid across workspace
# User options (change me)
# --------------- Setup options ---------------
workspace_limits = np.asarray([[0.37, 0.56], [-0.10,0.10], [0.3, 0.45]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.05
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)
num_calib_grid_pts

# ROS node initialization
rospy.init_node('perception', anonymous=True)

# Create move group of MoveIt for motion planning
moveit_commander.roscpp_initialize(sys.argv)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("panda_arm")
group.set_planner_id('RRTConnectkConfigDefault')
group.set_num_planning_attempts(10)
group.set_planning_time(5)
group.set_max_velocity_scaling_factor(0.5)

# Go through all the grid points and calculate the position of the checkerboard
tool = PoseStamped()
tool.header.frame_id = "panda_link0"
tool.pose.orientation.x = 0
tool.pose.orientation.y = 0
tool.pose.orientation.z = 0
tool.pose.orientation.w = 1

robot_pts = []
for iter in range(num_calib_grid_pts):
    group.set_start_state_to_current_state()
    group.clear_pose_targets()
    tool.pose.position.x = calib_grid_pts[iter,0]
    tool.pose.position.y = calib_grid_pts[iter,1]
    tool.pose.position.z = calib_grid_pts[iter,2]
    q = tf.transformations.quaternion_from_euler(0.2*np.random.uniform(0,1,1), 0.2*np.random.uniform(0,1,1), 0.2*np.random.uniform(0,1,1), axes='sxyz') # 0.35
    tool.pose.orientation.x = q[0]
    tool.pose.orientation.y = q[1]
    tool.pose.orientation.z = q[2]
    tool.pose.orientation.w = q[3]
    # (plan, fracktion) = group.compute_cartesian_path([group.get_current_pose().pose, tool.pose], eef_step=0.01, jump_threshold=0.0, avoid_collisions=True)
    group.set_pose_target(tool, end_effector_link='panda_link8')
    plan = group.plan()
    print("******************************************************")
    print("tool pose: ",tool)
    raw_input("Please enter:")
    # execute the plan, if execution failed, will return false and robot will switch to mode 4
    # uint8 ROBOT_MODE_OTHER=0
    # uint8 ROBOT_MODE_IDLE=1
    # uint8 ROBOT_MODE_MOVE=2
    # uint8 ROBOT_MODE_GUIDING=3
    # uint8 ROBOT_MODE_REFLEX=4
    # uint8 ROBOT_MODE_USER_STOPPED=5
    # uint8 ROBOT_MODE_AUTOMATIC_ERROR_RECOVERY=6
    res = group.execute(plan)
    time.sleep(2)
    if not res:
        # if fail, swich robot back to mode 2 so that next move can be executed
        # https://github.com/frankaemika/franka_ros/issues/69
        os.system("rostopic pub -1 /franka_control/error_recovery/goal franka_control/ErrorRecoveryActionGoal \"{}\"")
        continue
    # take color and depth images
    ret = 256
    attemps = 0
    while ret!=0 and attemps < 3:
        ret = os.system(r'./Photoneo'+r' '+timestamp+r'/ '+str(iter))
        attemps = attemps + 1
    if not ret:
        robot_pts.append(np.concatenate([calib_grid_pts[iter],q]))

np.save(timestamp+"/calibration_robot_pose_data.npy", np.array(robot_pts))
print("Collecting data Finished and data saved!")
