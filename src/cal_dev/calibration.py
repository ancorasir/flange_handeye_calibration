import pcl
import numpy as np
from pcl_msgs import *
# from sensor_stick.pcl_helper import *
import rospy 
from functools import reduce 
import time 
import tf
# # p_base 
# p_robot = np.array([[0.27265, -0.65591, 0.16760, 1],
#                [0.35291, -0.65217, 0.13476, 1],
#                [0.33833, -0.71059, 0.14230, 1],
#                [0.27451, -0.71246, 0.13478, 1]])
# p_robot = p_robot.transpose()


def get_p_robot(file_name):
    p_robot = []
    f = open(file_name,'r')
    reading = True
    i = 0
    while(reading):
        line = f.readline()
        if i >= 4:
            break
        if line.split(':')[0]=='index':
            i +=1
            f.readline()
            x = float(f.readline().split(':')[1].strip())
            y = float(f.readline().split(':')[1].strip())
            z = float(f.readline().split(':')[1].strip())
            p_robot.append([x, y, z, 1])
    
    p_robot = np.array(p_robot)
    return p_robot.transpose()

def hand_eye_calibrate(cloud_path):

    time_set = []
    for i in range(4):
        time_set.append(time.time())
        global p_camera
        # load obtained point cloud
        # cloud = pcl.load('./images/waypoint%s.ply'%(i+1))
        cloud_name = cloud_path+str(i+1)+'.pcd'
        print(cloud_name)
        cloud = pcl.load(cloud_path+str(i+1)+'.ply')
        time_set.append(time.time())
        # TODO: PassThrough Filter
        # roughly slicing for the point cloud 

        passthrough = cloud.make_passthrough_filter()
        # filtering in z axis from 380 to 435
        filter_axis = 'z'
        passthrough.set_filter_field_name(filter_axis)
        # axis_min = 380
        # axis_max = 435
        axis_min = 0.25
        axis_max = 0.47
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()
        pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+str(i)+'-'+str(axis_min)+'-'+str(axis_max)))
        time_set.append(time.time())

        
        # TODO: Statistical outlier filter
        # print("Statistical outlier filter:%d"%i)
        outlier_filter = cloud_filtered.make_statistical_outlier_filter()
        outlier_filter.set_mean_k(50)
        outlier_filter.set_std_dev_mul_thresh(1.0)
        cloud_filtered = outlier_filter.filter()
        time_set.append(time.time())
        pcl.save(cloud_filtered, "%s.pcd"%(cloud_path+str(i)+'-'+str(axis_min)+'-'+str(axis_max)+'_outlier'))
        
        # break 
        # TODO: Euclidean Clustering
        # print("Euclidean Clustering:%d"%i)
        white_cloud = cloud_filtered
        tree = white_cloud.make_kdtree()
        ec = white_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.0003)    # Set tolerances for distance threshold 
        ec.set_MinClusterSize(8000)
        ec.set_MaxClusterSize(1500000)   # as well as minimum and maximum cluster size (in points)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        tool_index = []
        time_set.append(time.time())
        # print(len(cluster_indices))
        # break 
        
        # select the tool0 plane has the largest number of points
        # print("Clustering:%d"%i)
        for cluster in cluster_indices:
            if len(cluster)>len(tool_index):
                tool_index = cluster
        tool0 = white_cloud.extract(tool_index)
        # pcl.save(tool0, "tool0_%s.pcd"%(i+1))
        time_set.append(time.time())


        # Ransac circle segmentation
        # tool0 = cloud_filter
        seg = tool0.make_segmenter()
        # seg.set_model_type(pcl.SACMODEL_PLANE)
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
        
        x0 = 100
        x1 = -100
        y0 = 100
        y1 = -100
        z0 = 100
        z1 = -100
        index_data = 0
        for data in outliers:
            points_list.append([data[0], data[1], data[2], 100])
        center = coefficients[:3]
        points_list.append([center[0], center[1], center[2], 100])
        #     if index_data == 0:
        #         x0 = x1 = data[0]
        #         y0=y1=data[1]
        #         z0=z1=data[2]

        #     if data[0] > x1:
        #         x1 = data[0]
        #     if data[0] < x0:
        #         x0 = data[0]
            
        #     if data[1] > y1:
        #         y1 = data[1]
        #     if data[1] < y0:
        #         y0 = data[1]

        #     if data[2] > z1:
        #         z1 = data[2]
        #     if data[2] < z0:
        #         x0 = data[2]
            
            
            


        
        print("CIRCLE")
        print("x:%f-%f  y:%f-%f  z:%f-%f"%(x0,x1,y0,y1,z0,z1))
        print()

        print("Analog Center:")
        print("x:%f  y:%f  z:%f"%( (x0+x1)/2, (y0+y1)/2, (z0+z1)/2))
        print("\n\n")
        

        
        tool0_c = pcl.PointCloud_PointXYZRGB()
        tool0_c.from_list(points_list)
        pcl.save(tool0_c, "%s.pcd"%(cloud_path+"tool_c"+str(i+1)))
        time_set.append(time.time())
        
        p_camera.append(coefficients[:3])
        # print("Cofficient")
        # print(coefficients)
        print("Center")
        print(coefficients[:3])
        time_set.append(time.time())
    return p_camera


# R = H[:3,:3]
# t = H[:3, 3]
# al, be, ga = tf.transformations.euler_from_matrix(R, 'sxyz')
# [16.695629119873047, -16.15144157409668, 413.2391662597656, 29.98028564453125, -0.015968363732099533, 0.019328217953443527, -0.9996856451034546]

def write_data_H(file_name, H):
  f = open(file_name, 'a')
  f.write("H\n")
  for i in range(4):
      f.write("%s\n"%(str(H[i])))
  f.write('\n')
  f.close()


def transfer(H):
  trans = H[2,0:3]
  rot_mat = H[0:3,0:3]
#   print(rot_mat)
  rot = tf.transformations.euler_from_matrix(rot_mat,'sxyz')
  return trans, rot 


if __name__ == '__main__' : 
    p_camera = []
    file_name = "/home/bionicdl/calibration_images/data.txt"
    p_robot = get_p_robot(file_name)
    # # print("Robot Pose")
    # # print(p_robot)
    cloud_path = "/home/bionicdl/calibration_images/im"
    p_camera2 =  hand_eye_calibrate(cloud_path)


    # print("P Camera")
    # print(p_camera)
    # print("P CAM 2")
    # print(p_camera2)
    # p_camera = [[-0.019356120377779007, -0.014356721192598343, 0.29263219237327576],
    #             [0.01416813675314188, -0.03466275334358215, 0.3105636239051819],
    #             [-0.0348646454513073, -0.053492434322834015, 0.39297932386398315],
    #             [0.01838613487780094, -0.0235764030367136, 0.3266702890396118]
    #             ]
    p_camera_ = np.matrix(np.concatenate((np.array(p_camera).transpose(), np.ones([1,4])),axis=0))
    print(p_camera_)
    p_camera_reverse = p_camera_.getI()
    # print(p_camera_reverse)
    H = np.matmul(p_robot, p_camera_reverse)
    trans, rot = transfer(H)
    print("Translation")
    print(trans)
    print("Rotate")
    print(rot)

    write_data_H(file_name, H)
    print("### Resut Origin")
    print("H")
    print(H)
    print("Transform")
    print(np.matmul(H,p_camera))
    print("robot")
    print(p_robot)
    print('cam')
    print(p_camera_)

    # p_camera_ = np.matrix(np.concatenate((np.array(p_camera).transpose(), np.ones([1,4])),axis=0))
    # # print(p_camera_)
    # p_camera_reverse = p_camera_.getI()
    # # print(p_camera_reverse)
    # H = np.dot(p_robot, p_camera_reverse)
    # write_data_H(file_name, H)

    # print("### Resut New")
    # print("H")
    # print(H)
    # print("Transform")
    # print(np.matmul(H,p_camera_))
    # print("robot")
    # print(p_robot)
    # print('cam')
    # print(p_camera_)

    

    



