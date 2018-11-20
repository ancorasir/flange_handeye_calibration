# Precision Test on Calibration Algorithm

## Testing on Rviz 

1. Change URDF according to the obtained transoformation matrix
2. Capture image through dpeth camera 
3. Observe the devation between point cloud and real position of the robot arm



### Test on Calibration board

1. Prepare a shap end-effector on the end of robot arm
2. Obtain the coordinate of a corner ponit on the calibration board from depth camera
3. Obtain the position of the corner point in robot pose (with consideration on the end effector) 
4. Manipulate the robot arm to go on the point, compare the deviation between pointing point to real point to get the deviation (quantatively)

