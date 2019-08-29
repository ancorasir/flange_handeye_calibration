import rospy
import time
import numpy as np
import sys 
sys.path.append("..")
from utils import transform, transform_ros
from geometry_msgs.msg import PoseArray, Pose, PoseStamped, Point, Quaternion
import unittest

class TransformTest(unittest.TestCase):
    def testPoseVec2ROS(self):
        test_time_init = 1000
        test_time = test_time_init
        while(test_time > 0):
            if test_time % 100 == 0:
                print("Running testing for PoseVec2ROS: %d/%d"%(test_time, test_time_init))
            test_time -= 1
            
            position = (np.random.rand(3) * np.random.randint(1000)).tolist()
            orientation = (np.random.rand(4)*2 - 1).tolist()
            
            position.extend(orientation)
            pose_vec = np.array(position)

            pose_ros = transform_ros.pose_from_vector(pose_vec)

            self.assertEquals(pose_vec[0], pose_ros.position.x)
            self.assertEquals(pose_vec[1], pose_ros.position.y)
            self.assertEquals(pose_vec[2], pose_ros.position.z)

            self.assertEquals(pose_vec[3], pose_ros.orientation.w)
            self.assertEquals(pose_vec[4], pose_ros.orientation.x)
            self.assertEquals(pose_vec[5], pose_ros.orientation.y)
            self.assertEquals(pose_vec[6], pose_ros.orientation.z)




    def testRos2PoseVec(self):
        test_time_init = 1000
        test_time = test_time_init
        while(test_time > 0):
            if test_time % 100 == 0:
                print("Running testing for ROS2PoseVec: %d/%d"%(test_time, test_time_init))

            position = (np.random.rand(3) * np.random.randint(1000)).tolist()
            orientation = (np.random.rand(4)*2 - 1).tolist()
            test_time -= 1

            pose_ros = Pose()
            pose_ros.position.x = position[0]
            pose_ros.position.x = position[1]
            pose_ros.position.x = position[2]
            pose_ros.orientation.w = orientation[0]
            pose_ros.orientation.x = orientation[1]
            pose_ros.orientation.y = orientation[2]
            pose_ros.orientation.z = orientation[3]

            pose_vec = transform_ros.pose_to_vector(pose_ros)

            self.assertEquals(pose_vec[0], pose_ros.position.x)
            self.assertEquals(pose_vec[1], pose_ros.position.y)
            self.assertEquals(pose_vec[2], pose_ros.position.z)

            self.assertEquals(pose_vec[3], pose_ros.orientation.w)
            self.assertEquals(pose_vec[4], pose_ros.orientation.x)
            self.assertEquals(pose_vec[5], pose_ros.orientation.y)
            self.assertEquals(pose_vec[6], pose_ros.orientation.z)
    
if __name__ == "__main__":
    unittest.main()
