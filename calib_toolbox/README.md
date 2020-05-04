## Framework

The program is divided into data collection and calibration parts.

Data collection include:

1. Image acquiring
2. Obtaining of robot pose from moveit
3. Obtaining of the 3DoF pose (just x,y,z position)  of center of the flange in camera frame ( which is the corresponding point of pose read moveit in the camera frame)
4. Write all data in the data file.

Calibration is just to read the poses from the data file and apply calibration.

 The two parts are separated which means ones we obtained enough data points and stored them in the data file, later experiment can be applied independently. (A demo of this part is provided at the end of **./src/cal_dev/calirbation.py**)

Besides, testing parts are implemented to first detect corner points on the calibration board, then move the robot to each corner points.

## Data Collection

To calibrate, data points are needed. Since the robot as well as the program have no prior knowledges on the camera, the calibration process requires the user to provide at least 4 points of robot poses separating as distant as possible in the view of camera.  The points must not be in the same plane and should be promised to have a clear and complete flange shown in the camera. The coordinate of a point should be writen manually in a data file in the form :

```
[x1, y1, z1]
[x2, y2, z2]
[xn, yn, zn]
```

The program will then read the coordinate from the file and apply calibration itself. The points provided by the user are named *flag_points*.

 The calibration process is divided into *Coarse Calibration* and *Fine Calibration*.



In *Coarse Calibration*, calibration are done with only the points provided by the user. This can provide a basic, while not precise enough result of calibration.  



For *Fine Calibration*, the program will obtain zig-zag path from between provided points. At first, all possible pairs of the *flag points* are made. Then, three extra points are interpolated with a given pair of points. Say we have two points :

```python
point_start = point_pair[0] # x1, y1, z1
point_end   = point_pair[1] # x2, y2, z2
```

Then, the orginal linearly interpolated points are obtained as:

```python
diff = point_end - point_start
interpolated_point1 = point_start  + 0.25 * diff
interpolated_point2 = point_start  + 0.50 * diff
interpolated_point3 = point_start  + 0.75 * diff
```

The return zig-zag path applies further turbulance to each of the linealry interpolated points by randomly adding shift between plus 3cm and minus 3cm to each of the three coordinates of a point independently.

Note that although intuitively moving robot to interpolated points should also have the flange showing clear in the camera frame, it is not always promised. Thus two criteria are added:

1. The number of the points on the flange plane after Euclidean Clustering
2. The number of points in the inlier set of the result of circle fitting.

The standard of the criteria is obtained from *flag points*.



## Calibration

For the calibration part, a ransac method is used.

In each iteration, 4 set of data (each with a 3DoF pose in the frame of robot and in the frame of camera) will be randomly picked from all recorded poses in the data file. Then there will be feasiblity check to see whether this points are in the same plane (or close to so). The matrix of the homogenous coordinate of robot pose and camera pose will be obtained, and then the determinant of the matrix of camera poses will be used to judge the feasiblity. If the determinant if smaller than 10 to the power of -6, the result will not be applicable and thus the check failed. A new random sets of points will be obtained if the check failed.

To select the final answer, criteria used here are simply the reporjection error of all recorded points in the data.

## Data Interface

```python
# Definition: /calib_toolbox/utils/piont_pair.py
class PointPair:
	def __init(self, index, p_robot, p_camera):
		p_robot = p_robot
		p_camera = p_camera
		extra_field = {} # extra information stored in dictionary

  def add_field(field, data) # add extra data into dictionary of extra_field

  def get_field(field, data) # get data from field

  def to_json()# convert information to a dictionary without numpy array

  def get_mat():
    return self.p_robot, self.p_cam

class PiontPairList:
  def __init__(self):
    self.pp_list = {}
    self.used_idx = [] # store used indicies when obtaining random list

  def read_old_data(file_path)

  def read(file_path)

  def write_data(file_path)

  def write_result(file_path, H)

  def get_len() # get total length of pp_list

  def get_mat(num_piont=4, idx_list=[]) # get random 4 by i matrix with required num_point or get 4 by i matrix with required index

  def get_mat_full() # get the full matrix of p_robot and p_camera

  def circle_extraction() # apply circle extraction and update to pplist
```

## Component Interface

```python

# All pose used for inter-system communication is represented in a numpy array:
# 	pose_vec = [position, quat] = [x, y, z, q_w, q_x, q_y, q_z], dtype=np.array
# Conversion to other format should be done in controller components

# Definition: /calib_toolbox/robot_controller
class RobotController:
  __init__() # set up necessary compoment, e.g. moveit group
  # Note that if there's need to convert uint metrics, convert it in the implementation of move() and get_curr_pose
  move(pose_vec) # move robot end-actuator to required pose
  get_curr_pose(): return pose_vec # return current pose of robot end-actuator

# Definition: /calib_toolbox/cam_controller
class CameraController:
	__init__() # set up necessary component, e.g. script path
  get_image(im_path) # obtain point clouds with camera, and save to required path

# Definition: /calib_toolbox/path_generator/path_generator.py
class PathGenerator:
  __init__() # generate path and store in some list
  get_waypint_next(): return pose_vec # return next pose for robot end-actuator, return empty numpy array if reaches the end of path

# Definition: /calib_toolbox/calibration/circle_fitting_main.py
class CircleExtractor:
  __call__(point_cloud_dir): return p_camera # return position of circle extracted if sucess, return empty numpy array if fails

# Definition: /calib_toolbox/calibration/calib_main.py
class Calibrator:
  __call__(point_pair_list): return H_final # input a PointPairList instance, return 4 by 4 H matrix in numpy array
```