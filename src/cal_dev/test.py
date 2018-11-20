import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs.msg
from std_msgs.msg import String
import rosbag
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tf 
import os 
import random
import cv2
import pcl

# NUM_ATTEMPS = 1
# PLANNER_NAME = "RRTConnect"
# MAX_PLAN_TIMES = 2

# # initialize moveit_commander and rospy node
# moveit_commander.roscpp_initialize(sys.argv)
# rospy.init_node('move_group_python_interface_tutorial',
#                   anonymous=True)

# # instantiate a RobotCommander object
# robot = moveit_commander.RobotCommander()
# # instantiate a Planning SceneInterface object
# scene = moveit_commander.PlanningSceneInterface()
# # Instantiate a Move Group Commander object
# group = moveit_commander.MoveGroupCommander("manipulator_i5")
# # publish trajectory to RViz
# display_trajectory_publisher = rospy.Publisher(
#                                       '/move_group/display_planned_path',
#                                  moveit_msgs.msg.DisplayTrajectory,
#                                       queue_size=20)


# # specify planner
# group.set_planner_id(PLANNER_NAME+'kConfig1')
# group.set_num_planning_attempts(1)
# group.set_planning_time(5)

# TODO: 


def get_transform(file_name):
    '''
        Get the transformation matrix 
    '''
    print(file_name)
    f = open(file_name, 'r')
    get = False
    while(True):
        line = f.readline()
        if line[0] == "H":
            break
    rows = []
    for i in range(3):
        rows.append(f.readline());
    
    H = []
    for row in rows:
        row = row.split('[')[2]
        row = row.split(']')[0]
        row = row.split(' ')
        row_num = []
        for item in row:
            if item!='':
                row_num.append(float(item))
        # H.append([float(row[1]), float(row[2]), float(row[4]), float(row[5])])
        H.append(row_num)
    return np.asmatrix(H)    
        


file_name = "/home/bionicdl/calibration_images/data.txt"
H = get_transform(file_name)
print(H)