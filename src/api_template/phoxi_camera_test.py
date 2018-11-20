#!/usr/bin/env python

import rospy
from phoxi_camera.srv import *
from std_srvs.srv import *
from time import sleep
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud

def depth_im_callback(msg):

    rospy.loginfo('Get Image')


document_path = "/home/bionicdl/calibration_images/"

if __name__ == '__main__':
    
    # initialize the ROS nod
    rospy.init_node('Photoneo_Control_Node')

    message = rospy.ServiceProxy('phoxi_camera/connect_camera', ConnectCamera)('1711004').message
    sleep(2)
    rospy.loginfo(message)
    # img_msg = rospy.ServiceProxy('phoxi_camera/get_frame', SaveFrame)(-1,document_path+'test_img.VTK').message
    im_id = rospy.ServiceProxy('phoxi_camera/trigger_image',TriggerImage)().id 
    im_msg = rospy.ServiceProxy('phoxi_camera/get_frame', GetFrame)(im_id).message
    rospy.loginfo(im_id)
    sub = rospy.Subscriber("phoxi_camera/depth_map", Image, depth_im_callback)
    
    im_id = rospy.ServiceProxy('phoxi_camera/trigger_image',TriggerImage)().id 
    rospy.loginfo(img_msg)
    rospy.spin()

