import pcl
import numpy as np
from sensor_stick.pcl_helper import *
from utils import *
from numpy.linalg import *
###########################################################################################
# calculate calibration data points from point cloud

data_path = "/home/bionicdl/photoneo_data/calibration_images/data_ransac10000"
index_list, p_camera_list, p_robot_list = read_data(data_path+"/data.txt", type="point_pair")

# flange segmentation
for index in index_list:
    try:
        cloud = pcl.load(data_path+'/im%s.ply'%(index[:-1]))
    except:
        print("pointcloud not existed: index %s!"%index)
        continue
    passthrough = cloud.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name (filter_axis)
    axis_min = 0.2
    axis_max = 0.5
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
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.0005)    # Set tolerances for distance threshold
    ec.set_MinClusterSize(2000)
    ec.set_MaxClusterSize(100000)   # as well as minimum and maximum cluster size (in points)
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
        height = cloud_array[:,2].min()
        if height < min_height:
            min_height = height
            tool_index = cluster
    tool0 = white_cloud.extract(tool_index)
    # plane segment
    seg = tool0.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    max_distance = 0.001
    seg.set_distance_threshold(max_distance)
    seg.set_MaxIterations(10000)
    seg.set_optimize_coefficients("true")
    seg.set_method_type(0)
    inliers, coefficients = seg.segment()
    flange = tool0.extract(inliers, negative=False)
    if len(inliers)>0:
        pcl.save(flange, "/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/tool0_%s.pcd"%(index[:-1]))
        print("Flange saved!")

# circle fitting
# 手动ransac 匹配模型
p_robot_used = []
maxD = 0.3/1000
R_FLANGE = 31.0/1000
detR = 0.001 
p_camera = []
for i in range(len(index_list)):
    index = index_list[i]
    try:
        tool0 = pcl.load("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/tool0_%s.pcd"%(index[:-1]))
    except:
        continue
    p_robot_used.append(p_robot_list[i])
    points = tool0.to_array()
    max_num_inliers = 0
    for k in range(10000):
        idx = np.random.randint(points.shape[0], size=3)
        A, B, C = points[idx,:]
        a = np.linalg.norm(C - B)
        b = np.linalg.norm(C - A)
        c = np.linalg.norm(B - A)
        s = (a + b + c) / 2
        R = a*b*c / 4 / np.sqrt(s * (s - a) * (s - b) * (s - c))
        if (R_FLANGE-R)>detR or R_FLANGE-R<0:
            continue
        b1 = a*a * (b*b + c*c - a*a)
        b2 = b*b * (a*a + c*c - b*b)
        b3 = c*c * (a*a + b*b - c*c)
        P = np.column_stack((A, B, C)).dot(np.hstack((b1, b2, b3)))
        P /= b1 + b2 + b3
        num_inliers = 0
        inliers = []
        outliers = []
        for point in points:
            d = np.abs(np.linalg.norm(point-P)-R)
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
        points_list.append([data[0], data[1], data[2], rgb_to_float([0,255,0])])
    for data in max_outliers:
        points_list.append([data[0], data[1], data[2], rgb_to_float([255,0,0])])
    tool0_c = pcl.PointCloud_PointXYZRGB()
    tool0_c.from_list(points_list)
    pcl.save(tool0_c, "/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/Tool0_c%s.pcd"%(i+1))
    p_camera.append(max_P)
    max_R
    max_P

p_robot_mat = np.array(p_robot_used).transpose()
p_camera_mat = np.array(p_camera).transpose()
np.save("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/p_robot_mat.npy",p_robot_mat)
np.save("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/p_camera_mat.npy",p_camera_mat)

##############################################################################
# Test interative calibration method deal with noisy data
from open3d import *
import numpy as np
import copy, tf
from calibration import get_calibrate

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def evaluate_calibration(s,t,H):
    sTt = copy.deepcopy(s)
    sTt.transform(H)
    HI = np.matrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    reg_p2p = registration_icp(sTt, t, 0.0003, HI, TransformationEstimationPointToPoint(),ICPConvergenceCriteria(max_iteration = 2000))
    R = reg_p2p.transformation[:3,:3]
    T = reg_p2p.transformation[:3,3]
    al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
    # print("xyz= [%2.2f, %2.2f, %2.2f]"%(T[0]*1000,T[1]*1000,T[2]*1000))
    # print("rpy= [%2.5f, %2.5f, %2.5f]"%(al,be,ga))
    return T*1000, np.array([al, be, ga])/3.14*180

# read point cloud for hand-eye calibration validation
target = read_point_cloud("/home/bionicdl/photoneo_data/20181217/aubo-i5-EndFlange_cropped_m.pcd")
source = read_point_cloud("/home/bionicdl/photoneo_data/20181217/tool0_5.pcd")
H_offset = np.matrix([[-1,0,0,0],[0,1,0,0],[0,0,-1,-0.006],[0,0,0,1]])
H_base_tool = tf.transformations.quaternion_matrix([-0.08580920098798522, -0.3893105864494028, 0.9148593368686363, 0.06408152657751885])
H_base_tool[:3,3] = np.array([0.27704067625331485, -0.573055166920657, 0.26205388882758757])
s = copy.deepcopy(source)
t = copy.deepcopy(target)
t.transform(H_offset)
t.transform(H_base_tool)

# read calibration points
p_robot_mat = np.load("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/p_robot_mat.npy")
p_camera_mat = np.load("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/p_camera_mat.npy")

calibrate = get_calibrate(4)
#######################################################
# method 1: calibration using all data points
H = calibrate(p_robot_mat, p_camera_mat)
R = H[:3,:3]
T = H[:3,3]
al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
print("xyz= %s"%(T.getT()))
print("rpy= %s %s %s"%(al,be,ga))
np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H.npy',H)
error_matrix = p_robot_mat - np.matmul(H[:3,:3],p_camera_mat) - np.tile(H[:3,3],[1,p_camera_mat.shape[1]])
error = np.mean( (np.sum((np.asarray(error_matrix))**2,axis=0))**(0.5) )
draw_registration_result(s, t, H)
###################################################
# method 2: least error
min_error = 10010
for i in range(1000):
    idx = np.random.choice(p_robot_mat.shape[1], 4,0)
    p_robot_mat_i = p_robot_mat[:,idx]
    p_camera_mat_i = p_camera_mat[:,idx]
    H_i = calibrate(p_robot_mat_i, p_camera_mat_i)
    error_matrix = p_robot_mat - np.matmul(H_i[:3,:3],p_camera_mat) - np.tile(H_i[:3,3],[1,p_camera_mat.shape[1]])
    error = np.mean( (np.sum((np.asarray(error_matrix))**2,axis=0))**(0.5) )
    # error =  np.mean( (np.sum((np.asarray(error_matrix))**2,axis=0)) ) # not good
    # print("Id:{} Iteration:{}  Error:{} mm".format(idx, i, error*1000))
    if error < min_error:
        min_error = error
        H1 = H_i

np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H1.npy',H1)
print("Id:{} Iteration:{}  min Error:{} mm".format(idx, i, min_error*1000))
evaluate_calibration(s,t,H1)
draw_registration_result(s, t, H1)
#######################################################
# method 3: Ransac
max_num = 0
min_error = 100
e_threshold = 0.0004
for i in range(1000):
    idx = np.random.choice(p_robot_mat.shape[1], 4,0)
    p_robot_mat_i = p_robot_mat[:,idx]
    p_camera_mat_i = p_camera_mat[:,idx]
    H_i = calibrate(p_robot_mat_i, p_camera_mat_i)
    error_matrix = p_robot_mat - np.matmul(H_i[:3,:3],p_camera_mat) - np.tile(H_i[:3,3],[1,p_camera_mat.shape[1]])
    error = (np.sum((np.asarray(error_matrix))**2,axis=0))**(0.5)
    num = sum(error<e_threshold)
    # print("Id:{} Iteration:{}  Error:{} mm".format(idx, i, error*1000))
    if num > max_num:
        selected = error<e_threshold
        max_num = num
        H_ransac = H_i
        min_error = np.mean(error[error<e_threshold])

np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H_ransac.npy',H_ransac)
print("Id:{} Iteration:{}  max_num:{} mean_error:{} mm".format(idx, i, max_num, min_error*1000))

p_robot_mat_s = p_robot_mat[:,selected]
p_camera_mat_s = p_camera_mat[:,selected]
H_s = calibrate(p_robot_mat_s, p_camera_mat_s)
np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H_s.npy',H_s)
error_matrix = p_robot_mat_s - np.matmul(H_s[:3,:3],p_camera_mat_s) - np.tile(H_s[:3,3],[1,p_camera_mat_s.shape[1]])
error = np.mean( (np.sum((np.asarray(error_matrix))**2,axis=0))**(0.5) )

evaluate_calibration(s,t,H_ransac)
evaluate_calibration(s,t,H_s)
#########################################################
# method 4: interative adding new points
error = []
xyz = []
rpy = []
for r in range(100):
    error_r = []
    xyz_r = []
    rpy_r = []
    idx = np.random.choice(p_robot_mat.shape[1], 63,0)
    p_robot_mat_i = p_robot_mat[:,idx[:5]]
    p_camera_mat_i = p_camera_mat[:,idx[:5]]
    error_i = 1000
    for i in range(5,len(idx)):
        for j in range(5):
            p_robot_mat_j = p_robot_mat_i
            p_camera_mat_j = p_camera_mat_i
            p_robot_mat_j = np.delete(p_robot_mat_j, j,1)
            p_camera_mat_j = np.delete(p_camera_mat_j, j,1)
            H_j = calibrate(p_robot_mat_j, p_camera_mat_j)
            xyz_j, rpy_j = evaluate_calibration(s,t,H_j)
            if 0<(np.sum(xyz_j**2))**(0.5)<error_i:
                error_i = (np.sum(xyz_j**2))**(0.5)
                H_i = H_j
                xyz_i = xyz_j
                rpy_i =rpy_j
                p_robot_mat_s = p_robot_mat_j
                p_camera_mat_s = p_camera_mat_j
        # print("Iteration:{}  Error:{} mm xyz:{} mm rpy:{}".format(i, error_i, xyz_i, rpy_i))
        error_r.append(error_i)
        xyz_r.append(xyz_i)
        rpy_r.append(rpy_i)
        # if error_i < 0.5:
        #     print("Terminated: calibration error is within 0.5 mm!")
        #     break
        p_robot_mat_i = np.concatenate((p_robot_mat_s, p_robot_mat[:,idx[i]].reshape([3,1])), axis=1)
        p_camera_mat_i = np.concatenate((p_camera_mat_s, p_camera_mat[:,idx[i]].reshape([3,1])), axis=1)
    print("Random:{}  Error:{} mm xyz:{} mm rpy:{}".format(r, error_i, xyz_i, rpy_i))
    error.append(error_r)
    xyz.append(xyz_r)
    rpy.append(rpy_r)

np.savez('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/results_photoneo.npz',error,xyz,rpy)
np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H_i.npy',H_i)
draw_registration_result(s, t, H_i)

data = np.load("/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/results_photoneo.npz")
error = data["arr_0"]
xyz = data["arr_1"]
rpy = data["arr_2"]

#####################################################
# iteratively remove bad points
p_robot_mat_i = p_robot_mat
p_camera_mat_i = p_camera_mat
for i in range(58):
    H_i = calibrate(p_robot_mat_i, p_camera_mat_i)
    xyz_i, rpy_i = evaluate_calibration(s,t,H_i)
    error_xyz = (np.sum(xyz_i**2))**(0.5)
    print("Iteration:{}  Error:{} mm xyz:{} mm rpy_i:{}".format(i, error_xyz, xyz_i, rpy_i))
    error_matrix = p_robot_mat_i - np.matmul(H_i[:3,:3],p_camera_mat_i) - np.tile(H_i[:3,3],[1, p_camera_mat_i.shape[1]])
    error = (np.sum((np.asarray(error_matrix))**2,axis=0))**(0.5)
    # print("Iteration:{}  Error:{} mm".format(i, np.mean(error*1000)))
    p_robot_mat_i = np.delete(p_robot_mat_i, np.argmax(error),1)
    p_camera_mat_i = np.delete(p_camera_mat_i, np.argmax(error),1)



np.save('/home/bionicdl/photoneo_data/calibration_images/data_ransac10000_valid/H_del.npy',H_i)
evaluate_calibration(s,t,H_del)
# results
# Iteration:49  Error:0.182933692566 mm
