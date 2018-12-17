ip=$1
roslaunch aubo_i5_moveit_config moveit_planning_execution.launch sim:=false robot_ip:=$ip&
sleep 5s
# python ./src/cal_dev/cal_main.py
# python ./src/cal_dev/cal_main.py
# kill -15 %1
# kill -15 %2
# python ./src/cal_dev/calibration.py
