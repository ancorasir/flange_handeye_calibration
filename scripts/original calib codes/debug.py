index=53
cloud = pcl.load(data_path+'/%s.ply'%(index))
passthrough = cloud.make_passthrough_filter()
filter_axis = 'z'
passthrough.set_filter_field_name (filter_axis)
axis_min = 500
axis_max = 1000
passthrough.set_filter_limits (axis_min, axis_max)
cloud_filtered = passthrough.filter()
pcl.save(cloud_filtered,"ss.pcd")

# TODO: Statistical outlier filter
outlier_filter = cloud_filtered.make_statistical_outlier_filter()
outlier_filter.set_mean_k(50)
outlier_filter.set_std_dev_mul_thresh(1.0)
cloud_filtered = outlier_filter.filter()
# TODO: Euclidean Clustering
white_cloud = XYZRGB_to_XYZ(cloud_filtered)
tree = white_cloud.make_kdtree()
ec = white_cloud.make_EuclideanClusterExtraction()
ec.set_ClusterTolerance(0.5)    # Set tolerances for distance threshold
ec.set_MinClusterSize(5000)
ec.set_MaxClusterSize(100000)   # as well as minimum and maximum cluster size (in points)
ec.set_SearchMethod(tree)
cluster_indices = ec.Extract()

# select the tool0 plane has the largest number of points

R_FLANGE = 31
j=0
points_list = []
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
    range_z_criter = 0 < (np.max(np.array(plane)[:,2])-np.min(np.array(plane)[:,2])) < R_FLANGE*2*0.5
    range_xy_criter
    (np.max(np.array(plane)[:,2])-np.min(np.array(plane)[:,2]))
    if 1:
    #if len(inliers)>0 and range_xy_criter[0] and range_xy_criter[1] and range_z_criter:
        for data in plane:
            points_list.append([data[0], data[1], data[2], rgb_to_float([0,255,0])])
        pcl.save(plane, "./cluster_%s.pcd"%(j))
        print("plane saved!")
    j = j+1

tool0_c = pcl.PointCloud_PointXYZRGB()
tool0_c.from_list(points_list)
pcl.save(tool0_c,"clusters.pcd")
