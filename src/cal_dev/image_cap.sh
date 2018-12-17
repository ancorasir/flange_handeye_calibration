num_interation="1"
im_path= $1
roslaunch phoxi_camera phoxi_camera_cal.launch num_of_iteration:="$num_interation" image_path:=$im_path&
sleep 10s

