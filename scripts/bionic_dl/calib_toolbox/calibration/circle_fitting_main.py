import pcl
import numpy as np
from numpy.linalg import *


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
        p_camera = self.circle_fitting(self, args[0], args[1], save_interm=False)
        return p_camera
