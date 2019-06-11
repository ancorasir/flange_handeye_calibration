if __name__ == "__main__":
    pass
    # test for franka robot
    from calib_toolbox.robot_controller.Franka_ROS import Franka_ROS_Controller
    robot_controller = Franka_ROS_Controller(None)
    curr_pose = robot_controller.get_current_pose()
    print(curr_pose)
    curr_pose[0] += 1
    print(curr_pose)
    robot_controller.move(curr_pose)

