import pcl
import numpy as np
from pcl_msgs import *
# from sensor_stick.pcl_helper import *
import rospy 
from functools import reduce 
import time 
import tf
# import multiprocessing as mp
from utils import *
from numpy.linalg import *
import os 

def circle_fitting(pointCloud_path, index=999):
    ''' Obtain the center of the circle at the flange of robot arm
        -Input: pointCloud_path (path of the point cloud), index(optional) 
    '''

    # data_path = os.path.dirname((pointCloud_path) + "/data.txt")
    # Get criteria from data file
    data_path = "/home/bionicdl/calibration_images/data.txt"
    if "flag" not in index:
        criteria = read_data(data_path, "criteria")
        # print("Get Criteria")
        # print(criteria)
    else:
        criteria = None 

    print("Path:")
    print(pointCloud_path)
    cloud_path = pointCloud_path
    pointCloud_path += ".ply"
    cloud = pcl.load(pointCloud_path)

    # Passthrough Filter 
    print("Passthrough Filter:%s"%str(index))
    passthrough = cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.25
    axis_max = 0.47
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    print("Path Through for %s"%str(index))
    # pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+"pass"+str(index)))

    
    # Statistical Outlier Filter
    print("Outlier Filter :%s"%str(index))
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.0)
    cloud_filtered = outlier_filter.filter()
    # pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+"ol"+str(index)))


    # Euclidean Clustering 
    print("Euclidean Clustering:%s"%str(index))
    white_cloud = cloud_filtered
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.001)    # Set tolerances for distance threshold 
    ec.set_MinClusterSize(2000)
    ec.set_MaxClusterSize(100000)   # as well as minimum and maximum cluster size (in points)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()
    tool_index = []
    
    
    min_height = 10010
    current_length = 0

    # Pick out cluseter on the top 
    for cluster in cluster_indices:
        cloud = white_cloud.extract(cluster)
        cloud_array = np.array(cloud)
        length = cloud_array.shape[0]
        height = cloud_array[:,2].min()
        if height < min_height:
            min_height = height 
            tool_index = cluster

    # Check number of points in the plane to match with criteria 
    if criteria and len(tool_index) < criteria[1] * 0.5:
        print("Error: Too Less Point in the plane")
        return False 
    tool0 = white_cloud.extract(tool_index)
    # pcl.save(tool0, "%s.pcd"%(cloud_path+"clustering"+str(index)))

    if "flag" in index:
        # Write Criteria For flag points 
        write_data(data_path, len(tool_index), "criteria:plane", type="vec" )



    # Ransac circle segmentation
    print("RANSAC :%s"%str(index))
    seg = tool0.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_CIRCLE3D)
    max_distance = 0.0002
    seg.set_distance_threshold(max_distance)
    seg.set_MaxIterations(10000)
    seg.set_optimize_coefficients("true")
    seg.set_method_type(6)
    inliers, coefficients = seg.segment()
    clc = tool0.extract(inliers, negative=False)
    outliers = tool0.extract(inliers, negative=True)
    points_list = []
    for data in clc:
        points_list.append([data[0], data[1], data[2], 0.5])
    
    index_data = 0
    for data in outliers:
        points_list.append([data[0], data[1], data[2], 255])
    center = coefficients[:3]
    points_list.append([center[0], center[1], center[2], 100])

    # Check inliers in the circle to match with criteria 
    # if criteria and (len(inliers) < int(criteria[0]*0.85) or int(len(inliers) < criteria[0]*1.3)):
    #     print("Error: Circle Size In-Correct")
    #     return False 

    if "flag" in index:
        # Write Criteria For flag points 
        write_data(data_path, len(inliers), "criteria:circle", type="vec" )
        
    
    tool0_c = pcl.PointCloud_PointXYZRGB()
    tool0_c.from_list(points_list)
    pcl.save(tool0_c, "%s.pcd"%(cloud_path+"tool_c"))
    

    return coefficients[:3]

def pose_diff(H1, H2):
    '''Compute the pose difference of 2 given transform matrix 
        -Input: H1, H2 for the two matrix to be compared
        -Output: float number of sum(H1*H2^(-1))
    '''
    H1 = np.mat(H1)
    H2 = np.mat(H2)
    diff = np.matmul(H1, H2.I)
    return diff

def get_calibrate(num_point):
    def calibrate_dlm(p_robot_mat, p_camera_mat):
        ''' Calibrate using 4 points '''
        # print("Det p_robot:{}".format(det(p_robot_mat)))
        # print("Det p_camera:{}".format(det(p_camera_mat)))
        H = np.matmul(p_robot_mat, p_camera_mat.getI())
        return H
        
    def calibrate_ls(p_robot_mat, p_camera_mat):
        ''' Calibtrate using least square solution points '''
        # H = np.dot(np.dot(np.dot(p_camera_mat.T,p_camera_mat).getI(), p_camera_mat.T), p_robot_mat)
        pass 
        # return H 

    def calibrate_svd(p_robot_mat, p_camera_mat):
        ''' Calibrate using svd'''
        pass 


    # def calibrate_16(p_robot_mat, p_camera_mat):
    #     ''' Calibration using 16 points '''
    #     pass     
    if num_point == 4:
        return calibrate_dlm
    elif num_point > 4:
        return calibrate_ls
    else:
        return False

        
def ransac(point_pair_list, num_point=4, max_iteration=10):
    ''' A RANSAC framework for calibration '''
    p_camera_total, p_robot_total = pp2mat(point_pair_list)
    calibrate = get_calibrate(num_point)

    def error_test(H):
        error = np.sum(p_robot_total - np.matmul(H, p_camera_total))
        return abs(error)

    len_list = len(point_pair_list)

    # Get maximum allowed iteration number 
    max_it_allowed = int(math.factorial(len_list)/(math.factorial(len_list-num_point)*math.factorial(num_point)) / 3)
    if max_iteration > max_it_allowed:
        max_iteration = max_it_allowed

    used_list = []
    min_error = 10010
    H_final = []
    for i in range(max_iteration):
        result = get_rand_list(used_list, point_pair_list, num_point=4)
        if result:
            p_camera_mat, p_robot_mat = result 
            H = calibrate(p_robot_mat, p_camera_mat)
            # The local error due to lose of precision in the float number need to be correct
            # local_err = np.sum(p_robot_mat - np.dot(H, p_camera_mat))
            # error = abs(error_test(H) - local_err  * len_list / num_point)
            error = error_test(H)
            # print(error)
            print("Times:{}  Error:{}".format(i, error))
            if error < min_error:
                min_error = error
                H_final = H

    return H_final, min_error 


def calibrate(data_path):
    point_pair_list = get_point_pair_list(data_path)
    num_point = 4
    calibrate = get_calibrate(num_point)
    used_list = []

    pp_temp = []
    data_list = []
    num_iterate = 100 
    H, error = ransac(point_pair_list)
    return H, error 
