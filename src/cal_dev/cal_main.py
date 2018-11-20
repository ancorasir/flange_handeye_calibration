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

NUM_ATTEMPS = 1
PLANNER_NAME = "RRTConnect"
MAX_PLAN_TIMES = 2
mode = "cal"

# FIXME: Unfinished part
def get_bound(x,y,z):
  return tf.transformations.quaternion_from_euler(x, y, z, axes='sxyz')

# def transfer(H):
#   trans = H[0:3,2]
#   rot_mat = H[0:3,0:3]
#   rot = tf.transformations.euler_from_matrix(R,'sxyz')
#   return trans, rot 

def copy_pose(original_pose):
  new_pose = geometry_msgs.msg.Pose()
  new_pose.position.x = original_pose.position.x
  new_pose.position.y = original_pose.position.y
  new_pose.position.z = original_pose.position.z

  new_pose.orientation.x = original_pose.orientation.x
  new_pose.orientation.y = original_pose.orientation.y
  new_pose.orientation.z = original_pose.orientation.z
  new_pose.orientation.w = original_pose.orientation.w

  return new_pose

# TODO
def get_path_all_around(current_pose):
  ''' 
  Get a path that mosty cover the configuration area of robot arm

  Target Representation:
    geometry_msgs -> Pose.msg:
      Point position (float64 x y z)
      Quaternion orientation (float64 w+xi+yj+zk)
  '''
  pass

# FIXME: Unfinished codes
def get_path_random(current_pose):
  ''' 
  Get a path that moving slightly around current position in a zig zag manner

  Target Representation:
      geometry_msgs -> Pose.msg:
        Point position (float64 x y z)
        Quaternion orientation (float64 w+xi+yj+zk)
  '''
  # bound = pose_bound(x, y, z, yaw, pitch, roll)# create pose bound
  path = []
  new_pose = current_pose
  for i in range(4):

    if (random.random() > 0.3):
      new_pose.position.x += 0.03 * random.random()

    if (random.random() > 0.4):
      new_pose.position.y += 0.03 * random.random()

    new_pose.position.z +=  0.03 * random.random()
      
    path.append(new_pose)
  return path

# FIXME: Unfinished codes
def get_path_zigzag(current_pose):
  ''' 
  Get a path that moving slightly around current position in a zig zag manner

  Target Representation:
      geometry_msgs -> Pose.msg:
        Point position (float64 x y z)
        Quaternion orientation (float64 w+xi+yj+zk)
  '''
  # bound = pose_bound(x, y, z, yaw, pitch, roll)# create pose bound
  path = []
  new_pose = copy_pose(current_pose)
  new_pose.position.x -= 0.01
  new_pose.position.y -= 0
  new_pose.position.z -= 0.01
  path.append(new_pose)


  new_pose = copy_pose(current_pose)
  new_pose.position.x += 0
  new_pose.position.y += 0.01
  new_pose.position.z -= 0.02
  path.append(new_pose)

  new_pose = copy_pose(current_pose)
  new_pose.position.x -= 0.01
  new_pose.position.y -= 0.01
  new_pose.position.z -= 0.03
  path.append(new_pose)

  new_pose = copy_pose(current_pose)
  new_pose.position.x += 0.01
  new_pose.position.y += 0.02
  new_pose.position.z -= 0
  path.append(new_pose)

  

  # new_pose = copy_pose(current_pose)
  # new_pose.position.x += 0.02
  # new_pose.position.y += 0.02
  # new_pose.position.z -= 0.04
  # path.append(new_pose)
  return path

def get_path_test(current_pose):
  new_pose = copy_pose(current_pose)
  path = []
  new_pose.position.y +=  0.03
  path.append(new_pose)

  new_pose = copy_pose(current_pose)
  new_pose.position.z += 0.06
  path.append(new_pose)
  # new_pose.position.z += 0.02
  # path.append(new_pose)
  print(path)
  return path 

def plan_execute(target, index, mode):
  plan_list = []
  print(target)
  group.set_start_state_to_current_state()
  group.clear_pose_targets()
  group.set_pose_target(target)
  for j in range(MAX_PLAN_TIMES):
    plan = group.plan()
    if len(plan.joint_trajectory.points) == 0:
      continue 
    plan_list.append(plan)
    rospy.sleep(0.5)
  print(group.get_current_pose().pose)
  if len(plan_list) == 0:
    print("Fail to get any plan")
    return


  group.execute(plan_list[0])
  rospy.sleep(5)
  if mode == "cal":
    im_name = "im" + str(index)
    status = os.system('sh /home/bionicdl/catkin_ws/src/fast_cal/image_cap.sh %s'%im_name)
    # print(status)

def write_pose2data(file_name, pose, index):
  f = open(file_name, 'a')
  f.write('index:%d\n'%index)
  f.write('position:\nx:%f\ny:%f\nz:%f\norientation:\nw:%f\nx:%f\ny:%f\nz:%f\n\n'
  %(pose.position.x,pose.position.y, pose.position.z, 
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z ))
  f.write('\n')
  f.close()

def get_target():
    '''
        Get coordinate of corner points from camera frame and convert it to robot base frame 
    '''
    img = cv2.imread("/home/bionicdl/calibration_images/imorigin.jpg")
    
    corners = []
    patternsize = tuple([8,11])
    count = 0
    # rows, cols, channels = img.shape
    # print(img.shape)
    ret, corners = cv2.findChessboardCorners(img, patternsize, count)
    # print(corners)
    img_new = cv2.drawChessboardCorners(img, patternsize, corners, ret)
    cv2.imwrite("/home/bionicdl/calibration_images/chess_plus.png", img_new) 
    cloud = pcl.load("/home/bionicdl/calibration_images/im.ply")    
    i = 0
    target = []
    for coord in corners:
        target.append(cloud.get_point(coord[0,0],coord[0,1]))
    return target


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

# initialize moveit_commander and rospy node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group_python_interface_tutorial',
                  anonymous=True)

# instantiate a RobotCommander object
robot = moveit_commander.RobotCommander()
# instantiate a Planning SceneInterface object
scene = moveit_commander.PlanningSceneInterface()
# Instantiate a Move Group Commander object
group = moveit_commander.MoveGroupCommander("manipulator_i5")
# publish trajectory to RViz
display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                 moveit_msgs.msg.DisplayTrajectory,
                                      queue_size=20)


# specify planner
group.set_planner_id(PLANNER_NAME+'kConfig1')
group.set_num_planning_attempts(1)
group.set_planning_time(5)

file_name = "/home/bionicdl/calibration_images/data.txt"
if mode == "cal":
  path_zigzag = get_path_zigzag(group.get_current_pose().pose)
  print("$$$$$$$$$$$$$$$$$$")
  print(path_zigzag)

  os.system('touch %s'%file_name)

  t = 1
  for target in path_zigzag:
    plan_execute(target,t, mode)
    write_pose2data(file_name, target, t)
    t = t+1

if mode == "test":
  im_name="im_test"
  status = os.system('sh /home/bionicdl/catkin_ws/src/fast_cal/image_cap.sh %s'%im_name)
  target_points = get_target()
  H = get_transform(file_name)
  print(H)
  print("######")
  for target in target_points:
    target = np.asmatrix(target)
    target_real = np.dot(target, H)
    print(target_real)
    print("######")
    
    # plan_execute(target, t, mode)


