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