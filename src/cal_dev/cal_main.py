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

# # mode: cal_corase, cal_fine, test, debug
mode = "test"
use_mp = True
use_auto_judge = False

killing_flag = False


def copy_pose(original_pose):
  ''' deep copy for pose object'''
  new_pose = geometry_msgs.msg.Pose()
  new_pose.position.x = original_pose.position.x
  new_pose.position.y = original_pose.position.y
  new_pose.position.z = original_pose.position.z

  new_pose.orientation.x = original_pose.orientation.x
  new_pose.orientation.y = original_pose.orientation.y
  new_pose.orientation.z = original_pose.orientation.z
  new_pose.orientation.w = original_pose.orientation.w

  return new_pose

def get_path_zigzag(path_, object_points):
  ''' 
    Generate a zig-zag path along provided points
      -Input: 
        * path: path made up of provided fix point 
        * object_points: list of 3D vector, representing points in the path 
      -Output:
        * zig-zag path
  '''
  path = copy.deepcopy(path_)
  zig_zag_path = []
  used_list = []

  num_points = len(object_points)
  for i in range(num_points):
    for j in range(i+1, num_points):
      used_list.append([i,j])

  for i,line in enumerate(used_list):
    print("Generating Points:%d/%d"%(i, len(used_list)))
    point_start = np.array(object_points[line[0]])
    point_end = np.array(object_points[line[1]])
    diff = point_end - point_start 

    path_point = []
    for k in [0.25, 0.5, 0.75]:
      point = point_start + k * diff 
      path_point.append(point)


    temp_zig_zag_path, null_object = get_fix_path(object_points=path_point, needTurbulance=True)
    for points in temp_zig_zag_path:
      zig_zag_path.append(points)

  return zig_zag_path

def get_fix_path(data_path=None, object_points=None, needTurbulance=False):
    ''' Add fix pose to the path
          -Input
            * data_path      [optional]: path of data file, if provided, will read target pose form file 
            * object_points  [optional]: a list of points, if provided, will add pose in the list the target path 
            * needTurbulance [optional]: if provided, will add random turbulance to path  
    '''
    path = []
    current_pose = group.get_current_pose().pose
    if data_path:
      object_points = read_data(data_path, type="target")
    

    for object in object_points:
      new_pose = copy_pose(current_pose)
      new_pose.position.x = object[0]
      new_pose.position.y = object[1]
      new_pose.position.z = object[2]
      if needTurbulance: 
        new_pose.position.x += (random.random()/10 - 0.05)*0.6
        new_pose.position.y += (random.random()/10 - 0.05)*0.6
        new_pose.position.z += (random.random()/10 - 0.05)*0.6

      if new_pose.position.z < 0.1:
        new_pose.position.z = 0.1
        

      path.append(new_pose)
    

    return path, object_points


def get_image(im_name):
  ''' Shoot a photo and save to the given path 
      - Input: im_name(path of the image)
  '''
  print("geting image with name:%s"%im_name)
  status = os.system('sh ./image_cap.sh %s'%im_name)
  # os.system("roslaunch phoxi_camera phoxi_camera_cal.launch num_of_iteration:=1 image_path:='%s'"%im_name)
  rospy.sleep(2)
  return status 

def plan_execute(target, mode, index=None):
  ''' Plan and execute for given target. Shoot and store image if in cal mode '''
  plan_list = []
  group.set_start_state_to_current_state()
  group.clear_pose_targets()
  group.set_pose_target(target)
  for j in range(MAX_PLAN_TIMES):
    plan = group.plan()
    if len(plan.joint_trajectory.points) == 0:
      continue 
    plan_list.append(plan)
    rospy.sleep(0.5)
  
  if len(plan_list) == 0:
    print("Fail to get any plan")
    return False

  # Excute the plan and motion
  print("Moving to target:")
  print(target)
  group.execute(plan_list[0])
  rospy.sleep(5)
  return 


def get_target(im_path=None, cloud_path=None):
    '''
        Get coordinate of corner points from camera frame and convert it to robot base frame 
    '''
    if not im_path:
      im_path = "/home/bionicdl/calibration_images/imorigin.jpg"
      cloud_path = "/home/bionicdl/calibration_images/im.ply"
    img = cv2.imread(im_path)
    corners = []
    patternsize = tuple([8,11])
    count = 0
    cv2.imwrite("/home/bionicdl/calibration_images/img_test.png", img) 
    ret, corners = cv2.findChessboardCorners(img, patternsize, count)
    img_new = cv2.drawChessboardCorners(img, patternsize, corners, ret)
    cv2.imwrite("/home/bionicdl/calibration_images/chess_plus.png", img_new) 
    cloud = pcl.load(cloud_path)
    target = []
    for coord in corners:
      # get the 3D coordinate of 2D point and add it to target list
      target.append(list(cloud.get_point(coord[0,0], coord[0,1])))
    return target 



class Worker(mp.Process):
    def __init__ (self, inQ, outQ, random_seed):
        super(Worker, self).__init__(target=self.start)
        self.inQ = inQ
        self.outQ = outQ
    
    def run (self):
        while True:
            task = self.inQ.get()  
            im_name, index, p_robot = task 
            if "flag" in index:
              need_judge = False
            else:
              global use_auto_judge
              need_judge = use_auto_judge
            p_camera = circle_fitting(im_name, index, need_judge=need_judge)
            print("Worker Get Result")
            if p_camera:
              self.outQ.put([index, p_camera, p_robot])  
            else:
              self.outQ.put(False)


def create_worker (num):
    for i in range(num):
        worker.append(Worker(mp.Queue(), mp.Queue(), np.random.randint(0, 10 ** 9)))
        worker[i].start()

def finish_worker ():
    for w in worker:
        w.terminate()

def move_shoot(target, mode):
  ''' Move to target and shoot an image (named after current timestamp) '''
  global use_auto_judge
  current_time = time.time()
  index = str(int(current_time)) + str(current_time - int(current_time)).replace(".","_")
  plan_execute(target, mode, index)  
  im_name = data_path + "im"
  # print(im_name)
  result = get_image(im_name)
  time.sleep(5)
  if use_auto_judge:
    input_str = "auto_mode"
  else:
    input_str = raw_input("Press 'Enter' to continue (input 'd' to delete current shoot or 'q' to quit calibration)")

  if input_str == "auto":
    use_auto_judge = True

  if input_str == "d":
    os.system('rm %s '%(im_name+".ply"))
    os.system('rm %s '%(im_name+".PCD"))
    return False
  elif input_str == "q":
    print("Killing the program")
    killing_flag = True
    return False
  else:
    os.system('mv %s %s'%(im_name+".ply", im_name+index+".ply"))
    os.system('mv %s %s'%(im_name+".PCD", im_name+index+".PCD"))
    im_name+= index
    current_pose = group.get_current_pose().pose
    p_robot = [current_pose.position.x,current_pose.position.y, current_pose.position.z, current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w]

    return im_name, index, p_robot

if __name__ == "__main__":
  NUM_ATTEMPS = 1
  PLANNER_NAME = "RRTConnect"
  MAX_PLAN_TIMES = 2
  # mode: cal, test, debug, move
  # mode = "move"
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
  # publish trajectory to RVizindex
  display_trajectory_publisher = rospy.Publisher(
                                        '/move_group/display_planned_path',
                                  moveit_msgs.msg.DisplayTrajectory,
                                        queue_size=20)


  # specify planner
  group.set_planner_id(PLANNER_NAME+'kConfig1')
  group.set_num_planning_attempts(1)
  group.set_planning_time(5)
  
  data_path = "/home/bionicdl/calibration_images/"  
  
  data_file = data_path + "data.txt"
  # Start Camera Node
  os.system("roslaunch phoxi_camera phoxi_camera.launch&")
  rospy.sleep(2)

  if mode == "debug":
    '''Debug mode: print and write current pose '''
    print(group.get_current_pose().pose)

  
  elif "cal" in mode:
    ## Calibrate mode 
    path, object_points = get_fix_path(data_path=data_path+"target.txt")

    # Create Multiple Process
    worker = []
    worker_num = 4
    create_worker(worker_num)
    working_worker_pipe = []

    # Get path for calibration 
    if mode == "cal_fine":
      zig_zag_path = get_path_zigzag(path, object_points)

    os.system('touch %s'%data_file)

    it = 0

    ## Coarse calibration
    original_use_auto_judge = use_auto_judge
    use_auto_judge = True
    if not use_mp:
      for i in range(len(path)):
        target = path[i]    
        im_name,index, p_robot = move_shoot(target, mode)
        p_camera = circle_fitting(im_name,  ("flag"+str(i)+"_"+index), need_judge=False)
        write_data(data_file, [p_robot, p_camera], index, type="point_pair")
    

    if use_mp:
      for i in range(len(path)):
        print("Working Process")
        print(working_worker_pipe)
        target = path[i]
        im_name,index, p_robot = move_shoot(target, mode)
        worker[i%worker_num].inQ.put([im_name, ("flag"+str(i)+"_"+index), p_robot])
        working_worker_pipe.append(i%worker_num)
      
        if len(working_worker_pipe) == worker_num:
          last_idx = working_worker_pipe[0]
          result = worker[last_idx].outQ.get()
          working_worker_pipe.remove(last_idx)
          index, p_camera, p_robot = result
          write_data(data_file, [p_robot, p_camera], index, type="point_pair")


      while len(working_worker_pipe) > 0 :
        last_idx = working_worker_pipe[0]
        result = worker[last_idx].outQ.get()
        working_worker_pipe.remove(last_idx)
        index, p_camera, p_robot = result
        write_data(data_file, [p_robot, p_camera], index, type="point_pair")

    use_auto_judge = original_use_auto_judge

    H_coarse, error_coarse = calibrate(data_file)
    print("Coarse Calibrate Finished")
    print(H_coarse)
    write_data(data_file, H_coarse ,"H_coarse", type="mat")
    trans, rot = H2trans_rot(H_coarse)
    write_data(data_file, trans, "Trans_coarse", type="vec")
    write_data(data_file, rot, "Rot_coarse", type="vec")

    H_final = copy.deepcopy(H_coarse)
    error_min = error_coarse


    ## Fine calibration 
    if mode == "cal_fine":
      for idx, target in enumerate(zig_zag_path):
        if killing_flag:
          sys.exit(0)
          break 

        print("##### Target: %d/%d #####"%(idx+1,len(zig_zag_path)))

        result = move_shoot(target, mode)
        if result:
          im_name, index, p_robot = result
        else:
          continue

        if not use_mp:
          p_camera = circle_fitting(im_name, index, need_judge=use_auto_judge)
          if p_camera:
            write_data(data_file, [p_robot, p_camera], index, type="point_pair")
            H, error = calibrate(data_file)
            # iteration of transformation matrix 
            if error < error_min:
              H_final = H
              error_min = error 
              print("New H")
              print(H)
              print(error_min)


        # multiple process 
        if use_mp:
          print("Working process:")
          print(working_worker_pipe)

          if len(working_worker_pipe) == worker_num:
              last_idx = working_worker_pipe[0]
              result = worker[last_idx].outQ.get()
              working_worker_pipe.remove(last_idx)
              getResult = True
          else:
              getResult = False

            
          if getResult and result:
            index, p_camera, p_robot = result
            write_data(data_file, [p_robot, p_camera], index, type="point_pair")
            H, error = calibrate(data_file)
            # iteration of transformation matrix 
            if error < error_min:
              H_final = H
              error_min = error 
              print("New H")
              print(H)
              print(error_min)
        
          worker[idx%worker_num].inQ.put([im_name, index, p_robot])
          working_worker_pipe.append(idx%worker_num)

      while len(working_worker_pipe) > 0 :
        last_idx = working_worker_pipe[0]
        result = worker[last_idx].outQ.get()
        working_worker_pipe.remove(last_idx)
        index, p_camera, p_robot = result
        write_data(data_file, [p_robot, p_camera], index, type="point_pair")

      finish_worker()

      write_data(data_file, H_final ,"H_final", type="mat")
      trans, rot = H2trans_rot(H_final)
      write_data(data_path, trans, "Trans", type="vec")
      write_data(data_path, rot, "Rot", type="vec")


  elif mode == "test":
      #Testing mode: obtain coordinate of a chessboard (assumed to be visible), and move the robot arm above

      im_name="im_test"
      # obatin image 
      result = get_image(im_name)
      rospy.sleep(10)
      # get target coordinate 
      target_cam = get_target()
      target_cam = np.matrix(np.concatenate((np.array(target_cam).transpose(), np.ones([1,len(target_cam)])),axis=0))
      # get transformation matrix
      H, error = calibrate(data_file)
      write_data(data_file, H,"H", type="mat")
      H = read_data(data_file, "H")
      print(H)
      # get target pose in robot base
      target_r = np.dot(H, target_cam)
      # transform from homogeneous coordinate to Cartesian coordinate
      target_r[:3,:] = target_r[:3,:]/target_r[3,:]
      target_r = target_r[:3,:]
      target_r = list(target_r.transpose())
      # execution for each task 
      for target in target_r:
        print(target)
        current_pose = group.get_current_pose().pose
        current_pose.position.x = target[0,0]
        current_pose.position.y = target[0,1]
        plan_execute(current_pose, mode)
        str = raw_input("Press to continue (press d to quit)Enter your input:")
        if str == "d":
          break 

