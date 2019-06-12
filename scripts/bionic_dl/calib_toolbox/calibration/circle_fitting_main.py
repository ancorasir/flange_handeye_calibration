import pcl
import numpy as np
from numpy.linalg import *
import os
import copy
from sensor_stick import *
from calib_toolbox.utils.pcl_helper import *

# TODO: Merge code from original implementation
def make_circle_extractor(cfg):
    if cfg.CIRCLE_FITTING.TYPE == "V1":
        return CircleFitting_V1()

class CircleFitting_V1:
    def __init__(self):
        pass

    # TODO: To be implemented
    def _feasibility_test(self, circle_plane, circle_line):
        return True

    def circle_fitting(self, point_cloud_path, index=999, need_judge=True, save_interm=False):
        # Load point cloud from given path
        # FIXME: For debug
        # print(point_cloud_path)
        # cloud = pcl.load("/home/bionicdl-Mega/repos/01.ply")
        cloud = pcl.load(point_cloud_path)

        # Passthrough Filter
        passthrough = cloud.make_passthrough_filter()
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = 0.25
        axis_max = 0.55
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        print("Path Through for %s" % str(index))
        # pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+"pass"+str(index)))

        # Statistical Outlier Filter
        print("Outlier Filter :%s" % str(index))
        outlier_filter = cloud_filtered.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(50)
        outlier_filter.set_std_dev_mul_thresh(1.0)
        cloud_filtered = outlier_filter.filter()
        # pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+"ol"+str(index)))

        # Euclidean Clustering
        print("Euclidean Clustering:%s" % str(index))
        white_cloud = cloud_filtered
        tree = white_cloud.make_kdtree()
        ec = white_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.001)  # Set tolerances for distance threshold
        ec.set_MinClusterSize(2000)
        ec.set_MaxClusterSize(100000)  # as well as minimum and maximum cluster size (in points)
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
            height = cloud_array[:, 2].min()
            if height < min_height:
                min_height = height
                tool_index = cluster

        tool0 = white_cloud.extract(tool_index)
        # pcl.save(tool0, "%s.pcd"%(cloud_path+"clustering"+str(index)))

        # RANSAC circle segmentation
        print("RANSAC :%s" % str(index))
        seg = tool0.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_CIRCLE3D)
        max_distance = 0.0002
        seg.set_distance_threshold(max_distance)
        seg.set_MaxIterations(50000)
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

        if need_judge in index:
            point_cloud_valid = self._feasibility_test(len(tool_index), len(inliers))
            if not point_cloud_valid:
                return False

        tool0_c = pcl.PointCloud_PointXYZRGB()
        tool0_c.from_list(points_list)
        # pcl.save(tool0_c, "%s.pcd"%(cloud_path+"tool_c"))

        # TODO: remove temporal point cloud and rename
        return coefficients[:3]

    def __call__(self, *args, **kwargs):
        point_cloud_dir = args[0]
        index = args[1]
        p_camera = self.circle_fitting(point_cloud_dir)
        return p_camera


class CircleFitting_V2:
    def __init__(self):
        pass

    def flange_extractor (self, point_cloud_path, output_path=None):
        try:
            cloud = pcl.load(point_cloud_path)
        except:
            print("pointcloud not existed: index %s!" % point_cloud_path)

        passthrough = cloud.make_passthrough_filter()
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = 500
        axis_max = 1000
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        # TODO: Statistical outlier filter
        outlier_filter = cloud_filtered.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(50)
        outlier_filter.set_std_dev_mul_thresh(1.0)
        cloud_filtered = outlier_filter.filter()
        # TODO: Euclidean Clustering
        white_cloud = XYZRGB_to_XYZ(cloud_filtered)
        tree = white_cloud.make_kdtree()
        ec = white_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.5)  # Set tolerances for distance threshold
        ec.set_MinClusterSize(2000)
        ec.set_MaxClusterSize(100000)  # as well as minimum and maximum cluster size (in points)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        tool_index = []
        # select the tool0 plane has the largest number of points
        min_height = 10010
        current_length = 0
        for cluster in cluster_indices:
            cloud = white_cloud.extract(cluster)
            cloud_array = np.array(cloud)
            length = cloud_array.shape[0]
            height = cloud_array[:, 2].min()
            if height < min_height:
                min_height = height
                tool_index = cluster
        tool0 = white_cloud.extract(tool_index)
        # plane segment
        seg = tool0.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        max_distance = 0.1
        seg.set_distance_threshold(max_distance)
        seg.set_MaxIterations(10000)
        seg.set_optimize_coefficients("true")
        seg.set_method_type(0)
        inliers, coefficients = seg.segment()
        flange = tool0.extract(inliers, negative=False)

        if not output_path:
            output_path = copy.deepcopy(point_cloud_path)
            if "pcd" in output_path:
                output_path.replace(".pcd", "_tool.pcd")
            elif "ply" in output_path:
                output_path.replace(".ply", "_tool.ply")

        if len(inliers) > 0:
            pcl.save(flange, output_path)
            print("Flange saved!")
            return output_path
        else:
            return None


    def ransac_circle_fitting(self, point_cloud_path):
        p_robot_used = []
        maxD = 0.3 / 1000
        R_FLANGE = 31.0 / 1000
        detR = 0.001
        p_camera = []
        # for i in range(len(index_list)):
        # index = index_list[i]
        try:
            # tool0 = pcl.load(
                # "/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/tool0_%s.pcd" % (
                # index[:-1]))
            tool0 = pcl.load(point_cloud_path)
        except:
            print("PointCloud with path %s not found"%(point_cloud_path))

        # p_robot_used.append(p_robot_list[i])
        points = tool0.to_array()
        max_num_inliers = 0
        for k in range(10000):
            idx = np.random.randint(points.shape[0], size=3)
            A, B, C = points[idx, :]
            a = np.linalg.norm(C - B)
            b = np.linalg.norm(C - A)
            c = np.linalg.norm(B - A)
            s = (a + b + c) / 2
            R = a * b * c / 4 / np.sqrt(s * (s - a) * (s - b) * (s - c))
            if (R_FLANGE - R) > detR or R_FLANGE - R < 0:
                continue
            b1 = a * a * (b * b + c * c - a * a)
            b2 = b * b * (a * a + c * c - b * b)
            b3 = c * c * (a * a + b * b - c * c)
            P = np.column_stack((A, B, C)).dot(np.hstack((b1, b2, b3)))
            P /= b1 + b2 + b3
            num_inliers = 0
            inliers = []
            outliers = []
            for point in points:
                d = np.abs(np.linalg.norm(point - P) - R)
                if d < maxD:
                    num_inliers += 1
                    inliers.append(point)
                else:
                    outliers.append(point)
            if num_inliers > max_num_inliers:
                max_num_inliers = num_inliers
                max_A = A
                max_B = B
                max_C = C
                max_R = R
                max_P = P
                max_inliers = np.array(inliers)
                max_outliers = np.array(outliers)
        points_list = []
        for data in max_inliers:
            points_list.append([data[0], data[1], data[2], rgb_to_float([0, 255, 0])])
        for data in max_outliers:
            points_list.append([data[0], data[1], data[2], rgb_to_float([255, 0, 0])])
        tool0_c = pcl.PointCloud_PointXYZRGB()
        tool0_c.from_list(points_list)

        output_path = point_cloud_path
        if "pcd" in output_path:
            output_path.replace(".pcd", "_valid.pcd")
        elif "ply" in output_path:
            output_path.replace(".ply", "_valid.ply")

        pcl.save(tool0_c, output_path)

        return np.array(max_P)


    def __call__(self, *args, **kwargs):
        point_cloud_dir = args[0]
        output_dir = self.flange_extractor(point_cloud_dir)
        if output_dir:
            p_camera = self.ransac_circle_fitting(output_dir)
            # p_camera = self.circle_fitting(self, args[0], args[1], save_interm=False)
            return p_camera
        else:
            return np.array([])


