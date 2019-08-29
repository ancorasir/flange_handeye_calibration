import pcl
import numpy as np
from pcl_helper import *
from utils import *
from numpy.linalg import *
###########################################################################################
# calculate calibration data points from point cloud
data_path = "/home/bionicdl/git-projects-py2/flange_handeye_calibration/data/2019-05-28/14:45:39"
NUM_POINTS = 75

index=2
cloud = pcl.load(data_path+'/%s.ply'%(index))

passthrough = cloud.make_passthrough_filter()
filter_axis = 'z'
passthrough.set_filter_field_name (filter_axis)
axis_min = 500
axis_max = 1000
passthrough.set_filter_limits (axis_min, axis_max)
cloud_filtered = passthrough.filter()
pcl.save(cloud_filtered, "cloud_filtered.pcd")

# TODO: Statistical outlier filter
outlier_filter = cloud_filtered.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
outlier_filter.set_std_dev_mul_thresh(1.0)
cloud_filtered = outlier_filter.filter()
pcl.save(cloud_filtered, "cloud_filtered_.pcd")

white_cloud = XYZRGB_to_XYZ(cloud_filtered)
ne = white_cloud.make_NormalEstimation()
tree = white_cloud.make_kdtree()
ne.set_KSearch(50)
ne.set_SearchMethod(tree)
normals = ne.compute()
nn = normals.to_array()

rg = white_cloud.make_RegionGrowing(ksearch=30,searchRadius=0)
rg.set_CurvatureThreshold(0.000002)
rg.set_InputNormals(normals)
rg.set_MaxClusterSize(1000000)
rg.set_MinClusterSize(3000)
rg.set_NumberOfNeighbours(30)
rg.set_SearchMethod(tree)
rg.set_SmoothModeFlag(True)
rg.set_SmoothnessThreshold(10.0 / 180.0 * 3.1415)
cluster_indices = rg.Extract()
print("Found %s clusters"%len(cluster_indices))
i = 0
for cluster in cluster_indices:
    cloud = cloud_filtered.extract(cluster)
    pcl.save(cloud, "cluster_%s.pcd"%(i))
    os.system("pcl_viewer "+"cluster_%s.pcd"%(i))
    i = i+1

#######################################################################

# TODO
# flange segmentation
R_FLANGE = 31
for index in range(NUM_POINTS):
    try:
        cloud = pcl.load(data_path+'/%s.ply'%(index))
    except:
        print("pointcloud not existed: index %s!"%index)
        continue
    passthrough = cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 500
    axis_max = 1000
    passthrough.set_filter_limits (axis_min, axis_max)
    cloud_filtered = passthrough.filter()
    # TODO: Statistical outlier filter
    outlier_filter = cloud_filtered.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(50)
    outlier_filter.set_std_dev_mul_thresh(1.0)
    cloud_filtered = outlier_filter.filter()
    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_filtered)
    tree = white_cloud.make_kdtree()
    ne = white_cloud.make_NormalEstimation()
    ne.set_KSearch(50)
    ne.set_SearchMethod(tree)
    normals = ne.compute()
    rg = cloud_filtered.make_RegionGrowing(ksearch=30,searchRadius=0)
    rg.set_CurvatureThreshold(0.000002)
    rg.set_InputNormals(normals)
    rg.set_MaxClusterSize(1000000)
    rg.set_MinClusterSize(3000)
    rg.set_NumberOfNeighbours(30)
    rg.set_SearchMethod(tree)
    rg.set_SmoothModeFlag(True)
    rg.set_SmoothnessThreshold(10.0 / 180.0 * 3.1415)
    cluster_indices = rg.Extract()
    tool_index = []
    print("Found %s clusters"%len(cluster_indices))
    # select the tool0 plane has the largest number of points
    for cluster in cluster_indices:
        cloud = white_cloud.extract(cluster)
        cloud_array = np.array(cloud)
        # plane segment
        seg = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        max_distance = 1
        seg.set_distance_threshold(max_distance)
        seg.set_MaxIterations(10000)
        seg.set_optimize_coefficients("true")
        seg.set_method_type(0)
        inliers, coefficients = seg.segment()
        plane = cloud.extract(inliers, negative=False)
        range_xy_criter = [ R_FLANGE*2*0.9 < (np.max(np.array(plane)[:,i])-np.min(np.array(plane)[:,i])) < R_FLANGE*2.2 for i in range(2)]
        range_z_criter = 0 < (np.max(np.array(plane)[:,2])-np.min(np.array(plane)[:,2])) < R_FLANGE*2*0.55
        if len(inliers)>0 and range_xy_criter[0] and range_xy_criter[1] and range_z_criter:
            pcl.save(plane, data_path+"/tool0_rg_%s.pcd"%(index))
            print("plane saved!")
            break

import os
for index in range(NUM_POINTS):
    try:
        os.system("pcl_viewer "+data_path+"/tool0_rg_%s.pcd"%(index))
    except:
        print("Pointcloud does not exist!")
