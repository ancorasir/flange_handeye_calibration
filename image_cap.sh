# num_iteration = $1
num_interation="1"
im_path="/home/bionicdl/calibration_images/$1"
roslaunch phoxi_camera phoxi_camera.launch& 
sleep 2s
echo "start"
roslaunch phoxi_camera phoxi_camera_cal.launch num_of_iteration:="$num_interation" image_path:=$im_path&
sleep 20s
echo "start killing"
kill -15 %1
kill -15 %2