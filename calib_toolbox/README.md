[TOC]

# Design note

### Framework

The program is divided into data collection and calibration parts.

Data collection include:

1. Image acquiring
2. Obtaining of robot pose from moveit 
3. Obtaining of the 3DoF pose (just x,y,z position)  of center of the flange in camera frame ( which is the corresponding point of pose read moveit in the camera frame)
4. Write all datas in the data file.

Calibration is just to read the poses from the data file and apply calibration. 

 The two parts are separated which means ones we obtained enough data points and stored them in the data file, later experiment can be applied independently. (A demo of this part is provided at the end of **./src/cal_dev/calirbation.py**)

Besides, testing parts are implemented to first detect corner points on the calibration board, then move the robot to each corner points. 

### Data Collection 

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



### Calibration 

For the calibration part, a ransac method is used. 

In each iteration, 4 set of data (each with a 3DoF pose in the frame of robot and in the frame of camera) will be randomly picked from all recorded poses in the data file. Then there will be feasiblity check to see whether this points are in the same plane (or close to so). The matrix of the homogenous coordinate of robot pose and camera pose will be obtained, and then the determinant of the matrix of camera poses will be used to judge the feasiblity. If the determinant if smaller than 10 to the power of -6, the result will not be applicable and thus the check failed. A new random sets of points will be obtained if the check failed. 

To select the final answer, criteria used here are simply the reporjection error of all recorded points in the data. 



# Code API 

### File Structure

**cal_main.py: **main program for calibration, used for collecting data and calibrating at the same time. The calibration part leverge functons imported from *calibration.py*

**calibration.py: **Tool functions for calibration. Can be used to calibrate directly if provided with a data file. 

**utils.py:** helper functions. 



### cal_main.py

**copy_pose:** Deep copy for pose object

- input: original_pose
- output: copied_pose

**get_path_zigzag**: Given target points provided by user manually (**flag points**), generate more points by interpolating between the* flag points* (three points are interpolated between any of the two flag points). 

- input: 
  - path_ : a list target points in the form of standard 6DoF pose object
  - object_points : a list target points in the form of 3DoF position vector
- output : 
  - zig_zag_path: a list of interpolated target points in the form of standard 6DoF pose object. The path include only the interpolated points. The original *flag points* are not included.

**get_fix_path**: Given a set of object points, generate a list tandard 6DoF pose object (with turbulance of plus minus 3cm if required Turbulance)

- input:
  - data_path\*: path of the data file, will read target from data file as object points if provided
  - object_points\*: a list target points in the form of 3DoF position vector, will use it directly as target if provided 
  - need_Trubulance\*: will apply plus minus 3 cm of turbulance to target points if provided 
- output:
  - path: a list target points in the form of standard 6DoF pose object
  - object_points:  a list target points in the form of 3DoF position vector

**get_image**: Shoot a photo and save to the given path 

- input: 
  - im_name: path of the image

**plan_execute**: generate a plane and move robot arm to required pose

- input: 
  - target:  target pose (standard 6 DoF pose) for execution
  - mode: current mode of calibration
- output: none



**move_shoot**: move robot arm to target pose, and shoot an image (which will be saved with name of the current timestamp)

- input:
  - target: target (standard 6 DoF pose) for execution
  - mode: current mode of calibration 
- output:
  - im_name: full path of the point cloud without post-fix ( e.g. "some_path/im123445", need add post-fix like ".pcd" or ".ply" when needed)
  - index: time stamp of the given image 
  - p_robot: 6 DoF pose of robot after execution

**get_target**: obtain an image, and return a list of 3D coordinates of corner points on the chessboard (camera frame)

- input:None 
- output: a list of 3D coordinate of corner (camera frame)



**parameters for main program**

- mode:
  - **cal_fine** : fine calibration with both *flag points* (points provided mannually by users and interpolated point)
  - **cal_coarse** : coarse calibration with only *flag points*
  - **test**: mode to walk through all corner points on a calibration board (chessboard)
- use_mp: Use multi-process if True. 
- use_auto_judge: Fully automatic calibration if True. 



### calibration.py

**circle_fitting**: apply circle fitting to point cloud in the given path 

- input: 
  - pointCloud_path (without post -fix, e.g.: "some_path/im_123456")
  - index\*: index of the point cloud (used to recognized flag points)
  - need_judge\*: will apply judge according to number of point cloud in the clustering plane if set to True
- output:
  - center_of_circle: 3DoF pose of center of the fitted circle
  - \* False if too less point on the plane



**calibrate:**

- input:
  - data_path: path for the data file 
  - num_point\*: number of points in the data 
  - max_iteration\*: maximum term of iteration 
- output:
  - H: transformation matrix
  - error: Reporjection error computed with all existing data points



**get_calibrate**: return a function for calibrate provided number of point

- input: num_point (number of points)

**Note:** currently the funciton only provide calibrate with direct linear method. Calibration using least square and SVD are remained to be finshed. 



**ransac**: a ransac framework for calibration 

- input: 
  - point_pair_list : a list of point pair
    - Here point pair is a specially defined data structure with properties:
      - p_robot  (3DoF)
      - p_camera (3DoF)
      - index : randomly assigned index 
    - The point_pair_list can be obtained with function *get_point_pair_list(data_path)* in *utils.py*
  - num_point\*: number of points to use for calibration 
  - max_iteration\*: maximum iteration for ransac
- output:
  - H: transformation matrix
  - error: Reporjection error computed with all existing data points



### utils.py

**class: PointPair**

- data structure to store corresponding 3DoF robot pose and camera pose 
- members:
  - p_robot
  - p_camera
  - index
- method:
  - __init\_\_: initialize given members

**pp2mat**: convert a list of point pairs to two matrix of p_robot and p_camera in homogeneous coordinate 

- Input: 
  - point_pair_list: a list of point pairs
- output
  - p_camera_mat
  - p_robot_mat



**write_data** : Interface for data writing 

- Input:

  - data_path: prcise path of the data file ("some_path/data.txt")

  - data_type: type of data 

    - "piont_pair" requires data to be [p_robot, p_camera]

    - "H":

    - "criteria": note that the two criterias is not written to the data file as a whole, but read from data file as a whole. i.e.

      ```python
      # Write
      write_data(path, criteria_plane, "circle:plane", type="criteria")
      write_data(path, criteria_line, "circle:line", type="criteria")
      # Read
      criteria = data_read(path, "criteria")
      ```

      A use case is in function **circle_feasibility_exam** in **calibration.py**

**read_data**: Interface for data reading

- Input:
  - data_path: precise path of the data file
  - data_type: type of data 
    - "piont_pair" return 3DoF pose of robot and camera (return : [p_robot, p_camera])
    - "H": return H matrix
    - "target": return a list of 3DoF target
    - "criteria": return [critera_plane, criteria_line] 



**Note: write_data and read_data is tightly coupled, change both if needed**



**get_point_pair_list**: read data and build a list of point pair

- Input: data_path (precise path of data)
- Output: point_pair_list



**print_point_pair_list**: print all point pairs in a point pair list:

- Input: point_pair_list



**get_rand_list**: obtain a random point_pair_list with number of required points

- Input:
  - point_pair_list
  - num_points\* : number of points required, default to 4
  - exact_list*: a list of number, will return a list of point pairs with required index in the list if provided
- output:
  - p_camera, p_robot

**Note: ** feasibility test is applied to ensure the point pair list of 4 has a determinant greater than 10-e06



**point_valid_check**: check if point is valid 

- input: 
  - point_pair_list
  - num_list\*: a list of index of required point_pairs
- Output:
  - p_camera, p_robot



  **Note:** test on determinant of matrix will be applied only when number of point pairs is 4.



# Phoxi_Camera Service API 

## Service 

### connect_camera

**Description:** Start *Phoxi_Control* and connect to the camera with required id. Suggest to sleep for 2 seconds to get camera connected. 

**Input:** 

string name: id of required camera

**Output:**

string message : indication of status; 'OK', 'Disconnected', 'Scanner not found'

bool success:



### get_device_list

**Description:**

**Input:** NaN

**Output:** 

int64 len: number of allowed cameras

string[] out: list of id of cameras

string message: indication of status

bool success:

### get_frame

**Description:** Get a frame and publish it

**Input:** int64 in : id of Input frame; negative number means get a new frame 

**Output:**

String message: indication of status

bool success

### save_frame

**Description:**

**Input:**

int64 in: id of scan returned by **trigger_image**, set it to negative if want to take a new image

string path:

**Output:**

string message: indication of status

bool success



### trigger_image

**Description:** Get an image from camera

**Input:** NaN

**Output:**

int32 id: id of the scan

string message: indicating status

bool success: boolean value indicate if success

### set_coordinate_space

refer to **set_transformation_matrix**

### set_transformation_matrix

**Description:** Apply transformation matrix to the camera. Dangerous to set value directly to the camera, better not use. 

**Input:**

geometry_msg/Transform transform: applied transform 

guint8 coordinate_space: 1 = CameraSpace, 2 = MountingSpace, 3 = MarkerSpace, 4 = RobotSpace, 5 = CustomSpace

bool set_value: if true then set coordinate_space to PhoXi Control

bool save_settings: if true then keep the settings 

**Output:**

string message: indication of status 

bool success



## Topics

### /phoxi_camera/confidence_map

**Description:** 

**Message Type:**

### /phoxi_camera/depth_map

**Description:** Depth image

**Message Type:** sensor_msgs/Image(Header, height, width, encoding, step, data[])

### /phoxi_camera/normal_map

**Description:** 

**Message Type:**

### /phoxi_camera/parameter_descriptions

**Description:** 

**Message Type:**

### /phoxi_camera/parameter_updates

**Description:** 

**Message Type:**

### /phoxi_camera/pointcloud

**Description:** Point cloud image

**Message Type:** sensor_msgs/PointCloud2 (Header, height, width, fields[], point_step, row_step, is_dense, data[] )

### /phoxi_camera/rgb_texture

**Description:** RGB image

**Message Type:** sensor_msgs/Image(Header, height, width, encoding, step, data[])

### /phoxi_camera/texture

**Description:** Gray scale image

**Message Type:** sensor_msgs/Image (Header, height, width, encoding, step, data[])