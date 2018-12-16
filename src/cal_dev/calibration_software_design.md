# Calibration Core

## Calibration Process

1. Display the list of available of camera and robot arm, and let user make a selection
2. User drag the robot arm to the middle part of the camera
3. Execute: Rough_calibrate
4. Execute: Precise_calibrate



### Rough Calibration

1. Moveit: Get current pose
2. Decide a zig-zag path (a set of points) according to current pose
3. Planning and execute all points on the zig-zag path
4. At each point: Phoxi_camera: get frame and store it in appointed directory
5. Execute calibration algorithm to get rough calibration results

### Precise Calibration

1. Reset Moveit config according to calibration resutls from Rough_calibration
2. Get the central coordinate and boundary coordinate of current perspective
3. Move the robot arm to central point of the vision 
4. Decide a all_around path (a set of points) according to perspective boundaries and central coordinate 
5. Planning and execute all points on the all_around path
6. At each point: Phoxi_camera: get frame and store in appointed directory
7. Execute calibration algorithm to get precise calibration results



### Current problem:

1. Coordinate representation in Moveit 
   Pose.msg:
   Point position (float64 x y z)
   Quaternion orientation (float64 w+xi+yj+zk)



1. RANSAC framework for calibration added
2. No longer take 4 points in a group. At first, the user must manually provide at least 4 points that have been ensured to work properly. The provided the points will be made into pairs and extra three points will be generated along the path from one point to anther point (with random noise of plus minus 3cm added  along each axis). 
3. A circle fitting will be performed after each image shooting. Only when the circle fitting turn out success, will the position of robot and camera be recorded in pair in the data file. 
4. The four points provided by user will be execute first to provided a rough estimate of transformation matrix as well as criteria to judge whether circle fitting succeeded. 
5. After the circle fitting of each point, a calibration using all available points using RANSAC will be performed. There will be an evaluation for the result. 
6. Calibration terminate when all points are used up or the result is good enough (according to evaluation). 

Criteria to judge whether a circle fitting success:

1. Number of points in the plane extracted from Euclidean clustering. (Taken account since sometimes the flange is not complete in the image, in this case, there’s no need to further performe circle fitting using RANSAC which is time comsuming)
2. Number of the points on the extracted circle (which reveals the radius of the circle)

The value of the two criteria comes from an average of corresponding values in the four points given by the user (with the assumption that circle fitting works well in point provided by the user, which is the usual case)

Criterial to evaluate the calibration result:

1. The overall reprojection error of computed transformation will be evaluated. 
2. Compare the normal vector of flange plane after projected to the robot plane with the value read from robot control. 

Two exisiting problem:

1. Since we need wait for the circle fitting process to finish before we can take the next point, the calibration becomes extermely slow
2. Some sampled points are bad. The robot arm may unpredictably hit the surrounding or fail to get the point. 

For the first problem, two solutions taken are:

1. Reduce the number of iteration in circle fitting since accuracy is not highly related to this part. 
2. Use a stack to implement a multiprocess control. 

No solutions have been found for the second problem, but the problem itself is not that significant so perhaps we shall just leave it there. 





###  标定流程

1. 从target.txt 获取用户事先定义的path point
2. 根据用于事先定义的path point采样和获取一系列zig-zag path
3. 预标定：利用用户事先定义的path point获取粗略标定的H和creteria（inlier个数和切面点数），这里inlier会在circle_fitting函数中被直接写入data里面，不会作为返回值返回
4. 精标定：
   1. 每一个新增加的点都需要在circle fitting的时候用criteria去判断。具体方法是circle fitting在启动的时候会尝试从data.txt里面读取criteria，如果读到了，就会利用criteria来进行判断
   2. 每一个新点新增加之后都要用RANSAC求一次H，并且得到H和error



### 精标定阶段的进程管理：

试图从working worker pipe中取出头上的index，如果freeworker长度为0，则timeout时长为一个大数（即要一直等到有执行结果位置）

1. 如果取出：
   1. 找到对应的index，p_robot (都是对应pipe头上的元素)，写入数据
2. 从free_worker_pipe中取出index，把这个index放入working_worker_pipe的末尾
3. index_pipe加上这个index，p_robot_pipe加上p_robot
4. 把当前任务assign给这个index对应的worker



Reamaining Work:

**Debug:**

1. Coarse calibrate only 
   1. Use of mutiprocess 
   2. criteria saving
2. Corase + Fine calibrate:
   1. Criteria reading
   2. Criteria using 
   3. Process management