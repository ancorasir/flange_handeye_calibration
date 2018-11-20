# Calibration Core

## Calibration Process

1. Display the list of available of camera and robot arm, and let user make a selection
2. User drag the robot arm to the middle part of the camera
3. Execute: Rough_calibrate
4. Execute: Precise_calibrate



### Rough Calibration

1. Moveit: Get current pose
2. Decide a zig-zag path (a set of points) according to current pose
3. Planning and execute all points on the zig-zag path
4. At each point: Phoxi_camera: get frame and store it in appointed directory
5. Execute calibration algorithm to get rough calibration results

### Precise Calibration

1. Reset Moveit config according to calibration resutls from Rough_calibration
2. Get the central coordinate and boundary coordinate of current perspective
3. Move the robot arm to central point of the vision 
4. Decide a all_around path (a set of points) according to perspective boundaries and central coordinate 
5. Planning and execute all points on the all_around path
6. At each point: Phoxi_camera: get frame and store in appointed directory
7. Execute calibration algorithm to get precise calibration results



### Current problem:

1. Coordinate representation in Moveit 
   Pose.msg:
   Point position (float64 x y z)
   Quaternion orientation (float64 w+xi+yj+zk)

