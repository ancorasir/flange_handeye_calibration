#!/usr/bin/env python
import numpy as np
import cv2,  time, os
import pyrealsense2 as rs

timestamp=time.strftime('data/%Y-%m-%d/%H:%M:%S',time.localtime(int(round(time.time()*1000))/1000))
folder=time.strftime('data/%Y-%m-%d',time.localtime(int(round(time.time()*1000))/1000))
if not os.path.exists(timestamp):
    os.makedirs(timestamp)

# TODO: Construct 3D calibration grid across workspace
# User options (change me)
# --------------- Setup options ---------------
workspace_limits = np.asarray([[-0.05, 0.0+0.25], [-0.83,-0.57], [0.17, 0.37]]) # Cols: min max, Rows: x y z (define workspace limits in robot coordinates)
calib_grid_step = 0.07
gridspace_x = np.linspace(workspace_limits[0][0], workspace_limits[0][1], 1 + (workspace_limits[0][1] - workspace_limits[0][0])/calib_grid_step)
gridspace_y = np.linspace(workspace_limits[1][0], workspace_limits[1][1], 1 + (workspace_limits[1][1] - workspace_limits[1][0])/calib_grid_step)
gridspace_z = np.linspace(workspace_limits[2][0], workspace_limits[2][1], 1 + (workspace_limits[2][1] - workspace_limits[2][0])/calib_grid_step)
calib_grid_x, calib_grid_y, calib_grid_z = np.meshgrid(gridspace_x, gridspace_y, gridspace_z)
num_calib_grid_pts = calib_grid_x.shape[0]*calib_grid_x.shape[1]*calib_grid_x.shape[2]
calib_grid_x.shape = (num_calib_grid_pts,1)
calib_grid_y.shape = (num_calib_grid_pts,1)
calib_grid_z.shape = (num_calib_grid_pts,1)
calib_grid_pts = np.concatenate((calib_grid_x, calib_grid_y, calib_grid_z), axis=1)

# Go through all the grid points and calculate the position of the checkerboard

import urx,tf
robot_ip = "192.168.1.27" # The remote robot_ip
rob = urx.Robot(robot_ip)
pose = rob.get_pose()
pose.set_pos((workspace_limits[0,0], workspace_limits[1,0],workspace_limits[2,0]))
rob.movex('movel', pose, acc=0.5, vel=0.5)

robot_pts = []

for iter in range(num_calib_grid_pts):
    x = calib_grid_pts[iter,0]
    y = calib_grid_pts[iter,1]
    z = calib_grid_pts[iter,2]
    pose.set_pos((x, y, z))
    # tf.transformations.euler_from_matrix(rob.get_pose().get_array()[:3,:3])
    roll = np.random.uniform(-0.3,0.3)
    pitch = np.random.uniform(-0.3,0.3)
    pose.set_orient(tf.transformations.euler_matrix(roll,pitch,0)[:3,:3])
    rob.movex('movel', pose, acc=0.3, vel=0.3)
    time.sleep(2)
    robot_pts.append(rob.get_pose().get_array())
    # take color and depth images
    os.system(r'./Photoneo'+r' '+timestamp+r'/ '+str(iter))
    time.sleep(5)

np.savez(timestamp+"/calibration_robot_pose_data.npz", np.array(robot_pts))
print("Collecting data Finished and data saved!")

rob.close()
