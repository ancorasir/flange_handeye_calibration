#!/usr/bin/env python

import rospy
from phoxi_camera.srv import *
from std_srvs.srv import *
from time import sleep
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointCloud
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import numpy as np 
import pcl

def gray_im_callback(msg):
#   ''' 
#     1. Find the x,y coordinate of corner points for the chessboard
#     2. Find the x,y,z coordiate of corner points of the chessboard
#     3. Write x,y,z coordinate into a file 
#   '''
#     # sleep(5)
#     img = cv2.imread("/home/bionicdl/calibration_images/origin.jpg")
    
#     corners = []
#     patternsize = tuple([6,7])
#     count = 0
#     # rows, cols, channels = img.shape
#     print(img.shape)
#     ret, corners = cv2.findChessboardCorners(img, patternsize, count)
#     img_new = cv2.drawChessboardCorners(img, patternsize, corners, ret)
#     cv2.imwrite("/home/bionicdl/calibration_images/chess_plus.png", img_new)
#     cloud = pcl.load("/home/bionicdl/calibration_images/im.ply")
#     # get x y z coordinate according to x y
    img = cv2.imread("/home/bionicdl/calibration_images/imorigin.jpg")
    
    corners = []
    patternsize = tuple([8,11])
    count = 0
    # rows, cols, channels = img.shape
    # print(img.shape)
    ret, corners = cv2.findChessboardCorners(img, patternsize, count)
    # print(corners)
    img_new = cv2.drawChessboardCorners(img, patternsize, corners, ret)
    cv2.imwrite("/home/bionicdl/calibration_images/chess_plus.png", img_new) 
    cloud = pcl.load("/home/bionicdl/calibration_images/im.ply")    
    i = 0
    for coord in corners:
        # print(corners)
        i+=1
        write_corner(file_name, coord, i, cloud)

     
    
def write_corner(file_name, coord, index, cloud):
    f = open(file_name, 'a')
    if index == 1:
        f.write("Corner Points:\n")
    point = cloud.get_point(coord[0,0], coord[0,1])
    # print("coord:%f %f"%(coord[0,0], coord[0,1]))
    # print("Point: %f,%f,%f"%(point[0], point[1], point[2]))
    
    f.write('index:%d\n'%index)
    f.write('%f,%f,%f'%(point[0], point[1], point[2]))
    f.write('\n')
    f.close()



document_path = "/home/bionicdl/calibration_images/"
file_name= "/home/bionicdl/calibration_images/data.txt"
if __name__ == '__main__':
    # initialize the ROS nod
    rospy.init_node('chessboard_cal_node')
    sub = rospy.Subscriber("phoxi_camera/texture", Image, gray_im_callback)
    rospy.spin()
    # img = cv2.imread("/home/bionicdl/calibration_images/imorigin.jpg")
    
    # corners = []
    # patternsize = tuple([8,11])
    # count = 0
    # # rows, cols, channels = img.shape
    # # print(img.shape)
    # ret, corners = cv2.findChessboardCorners(img, patternsize, count)
    # # print(corners)
    # img_new = cv2.drawChessboardCorners(img, patternsize, corners, ret)
    # cv2.imwrite("/home/bionicdl/calibration_images/chess_plus.png", img_new) 
    # cloud = pcl.load("/home/bionicdl/calibration_images/im.ply")    
    # i = 0
    # for coord in corners:
    #     # print(corners)
    #     i+=1
    #     write_corner(file_name, coord, i, cloud)



