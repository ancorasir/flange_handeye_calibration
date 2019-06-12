# Bionic_DL
The package include some data interfaces for robot development and calibration 

# Re-Development
## Robot Controller Class:
Methods to be implement for robot controller class:
* get_curr_pose(): return a numpy array of 7d vector of current pose of robot end-actuator 
* move(pose_vec): input a numpy array of 7d vector of current pose, move to required position  

## Camera Controller Class:
Method to be implement for camera controller class:
* get_image(image_name): move the camera to shoot an RGBD iamge and save as image name 


