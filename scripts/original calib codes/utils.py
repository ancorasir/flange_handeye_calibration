import numpy as np 
import os 
import random 
import sys 
from numpy.linalg import *
import random 
import itertools
import math 
import tf 

class PointPair:
    ''' data strcture to store corresponding robot pose and camera pose 
        - Member:
            * p_robot: 3D vector of position of robot 
            * p_camera: 3D vector of position of robot 
    '''
    def __init__(self, p_robot, p_camera, index=999):
        self.p_robot    = p_robot
        self.p_camera   = p_camera
        self.index  = index

        
def pp2mat(point_pair_list):
    ''' Convert point pair to matrices of homogenous coordinate for robot and camera
            - Input: a point pair list containing several point pair
            - Output: two n*4 matrix for camera matrix and robot matrix 
    '''
    p_camera_mat = []
    p_robot_mat = []
    length = len(point_pair_list)
    for i in range(length):
        p_camera_mat.append(point_pair_list[i].p_camera)
        p_robot_mat.append(point_pair_list[i].p_robot)
    p_robot_mat = np.matrix(np.concatenate((np.array(p_robot_mat).transpose(), np.ones([1,length])),axis=0))
    p_camera_mat = np.matrix(np.concatenate((np.array(p_camera_mat).transpose(), np.ones([1,length])),axis=0))
    return p_camera_mat, p_robot_mat 


def read_data(data_path, type):
    ''' Read data from given data file 
        - Input: 
            * data_path : precise path of the data file
            * type: [selection]
                "point_pair" (return 3D vector of position of robot and camera)
                "H" (return matrix H)
                "target" (return target coordiante)
                "criteria" (return criteria for circle and plane)
        - Output:
            * Two 1*3 vector for position of camera and robot relatively
    ''' 
    f = open(data_path, 'r')

    def str2array(line):
        '''convert string '[a,b,c,d]' to numpy array [a,b,c,d] '''
        print(line)
        line = line.split("[")[1].split("]")[0].split(",")
        if line[-1] == " ":
            line = line[:-1]
        return np.array(line, dtype=np.double)

    if type == "point_pair":
        index_list = []
        p_robot_list = []
        p_camera_list = []
        while(True):
            line = f.readline()
            if line:
                if "p_robot" in line:
                    index = line.split(":")[1]
                    index_list.append(index)
                    p_robot = str2array(f.readline())[:3]
                    f.readline()
                    p_camera = str2array(f.readline())
                    p_robot_list.append(p_robot)
                    p_camera_list.append(p_camera)
            else:
                break 
        f.close()
        return index_list, p_camera_list, p_robot_list
    elif type == "H":
        while(True):
            line = f.readline()
            if "H" in line:
                break 
        H = []
        for i in range(4):
            row = str2array(f.readline())
            H.append(row)
        f.close()
        return np.mat(H)
    elif type == "target":
        target = []
        while(True):
            line = f.readline()
            if line:
                target.append(list(str2array(line)))
            else:
                f.close()
                return target
    elif type == "criteria":
        num_criteria = 0
        criteria_circle = 0
        criteria_plane = 0
        min_criteria_circle = 9999999
        min_criteria_plane = 9999999
        flag = 0
        while(True):
            line = f.readline()
            if line:
                if "plane" in line:
                    flag += 1
                    criteria_plane = int(f.readline())
                    f.readline()
                    f.readline()
                    f.readline()
                    criteria_circle = int(f.readline())
                    if criteria_plane < min_criteria_plane:
                        min_criteria_plane = criteria_plane
                    if criteria_circle < min_criteria_circle:
                        min_criteria_circle = criteria_circle
            else:
                if  criteria_circle > 0:
                    # return [int(criteria_circle/flag), int(criteria_plane/flag)]
                    return [min_criteria_circle, min_criteria_plane]
                else:
                    return None 
                f.close() 

                


        



def point_valid_check(point_pair_list_, num_list=[]):
    '''' Check if given points are in the same plane 
            - Input: 
                * point_pair_list
                * num_list : selective, will pick corresponding point pairs from pp_list if given
            - Output: 
                * p_camera_mat 
                * p_robot_mat (both in Homogeneous coordinate) 
                * or False if given matrix is not allowed
    '''
    
    point_pair_list = []
    if len(num_list) > 0:
        for idx in num_list:
            point_pair_list.append(point_pair_list_[idx])
    else:
        point_pair_list = point_pair_list_

    p_camera, p_robot = pp2mat(point_pair_list)

    if len(num_list) > 4:
        return p_camera, p_robot


    if abs(det(p_camera)) > math.pow(10,-6):
        return p_camera, p_robot
    else:
        print(abs(det(p_camera)))
        return False




def print_point_list(point_pair_list):
    ''' Codes for displaying inform in point pair list 
        - Input: 
            * pp_list: a list of point pair 
    '''
    for i in range(len(point_pair_list)):
        pp = pp_list[i]
        # the below code only work in python3, so it is commented to avoid generating error 
        print("r:{} c:{} i:{}\n".format(pp.p_robot, pp.p_camera, pp.index))



def get_point_pair_list(data_path):
    ''' Obtain a list of point pairs form data file 
        - Input:
            * data_path: full path to the data file 
        - Output:
            * a list of point pair 
    '''
    pp_list = [] # abbr. for point pair list 

    index_list, p_camera_list, p_robot_list = read_data(data_path, type="point_pair")
    for i in range(len(index_list)):
        index = index_list[i]
        p_cameara = p_camera_list[i]
        p_robot = p_robot_list[i]
        pp_list.append(PointPair(p_robot, p_cameara, index))

    for i in range(len(pp_list)):
        pp_list[i].index = i 

    return pp_list



def get_rand_list(point_pair_list, num_point=4, exact_list=[]):
    '''Select random points that is not in used list 
        - Input: 
            * pp_list :  point_pair_list
            * num_point: number of points to use (minimum 4)
        - Output: 
            * p_robot_mat
            * p_camera_mat 
            * or False, when there's no valid point pairs
    '''
    flag_fail_time = 0
    rand_list = []

    while True: 
        if not exact_list:
            num_used = []
            exact_list = []
            while(True):
                while(True):
                    rand_int = random.randint(0, len(point_pair_list)-1)
                    if rand_int not in num_used:
                        num_used.append(rand_int)
                        exact_list.append(rand_int)
                        break 

                if len(exact_list) == num_point:
                    break 
                    # if exact_list not in used_list:
                    #     used_list.append(exact_list)
                    #     break
                    # else:
                    #     exact_list = []
        # print(exact_list)
        result = point_valid_check(point_pair_list, num_list=exact_list)

        if result:
            return result
        else:
            flag_fail_time += 1
            if flag_fail_time > 10:
                return False
            exact_list = []
            # print("Failing Times:{}".format(flag_fail_time))
            continue


def write_data(data_path, data, title, type="mat"):
    ''' Write data to file 
        - Input:
            * data_path: path of the data file
            * data: data to be write
            * title: header title of the data 
            * type: [selective] type of data to be write
                "mat":matrix
                "vec": vector
                "point_pair": point pair   
    '''

    f = open(data_path, 'a')
    if type == "mat":
        f.write(title+"\n")
        data_mat = np.array(data)
        row, col = data_mat.shape

        for r in range(row):
            line = "["
            for c in range(col):
                line += str(data_mat[r,c])+", "
            line += "]\n"   
            f.write(line)
        


    elif type == "vec":
        f.write(title+"\n")
        f.write("%s\n"%(str(data)))

    elif type == "point_pair":
        index = title
        p_robot, p_camera = data
        title_robot = "p_robot 6-DoF index:"+str(index)
        title_camera = "p_camera 3-DoF index:"+str(index)
        f.write(title_robot +"\n")
        f.write("%s\n"%(str(p_robot)))
        f.write(title_camera+"\n")
        f.write("%s\n"%(str(p_camera)))
    f.write('\n \n')
    f.close()


def H2trans_rot(H):
    ''' Obtain rotation and translation matrix from H 
        -Input: H
        -Output: trans, rot
    '''
    trans = list(H[0:3,3])
    rot_mat = H[0:3,0:3]

    rot = tf.transformations.euler_from_matrix(rot_mat,'sxyz')
    return trans, rot 