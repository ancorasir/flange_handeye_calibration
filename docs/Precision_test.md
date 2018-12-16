# Precision Test on Calibration Algorithm

## Testing on Rviz 

1. Change URDF according to the obtained transoformation matrix
2. Capture image through dpeth camera 
3. Observe the devation between point cloud and real position of the robot arm



### Test on Calibration board

1. Prepare a shap end-effector on the end of robot arm
2. Obtain the coordinate of a corner ponit on the calibration board from depth camera
3. Obtain the position of the corner point in robot pose (with consideration on the end effector) 
4. Manipulate the robot arm to go on the point, compare the deviation between pointing point to real point to get the deviation (quantatively)



### 测试

1. 走棋盘格测试定位误差（程序开发部分已经完成），具体精度：画点，人工测量。
2. 利用相机视野不同区域标定，比较标定结果 (新模式开发：手动定点，读取位置并控制机械臂)
3. 相机视野同一区域不同标定点，比较标定结果的误差（需要测试多组数据）
4. 使用不同数量的点标定，比较标定结果（8组，16组点，超定方程SVD解，尚未开发）

### 后续功能开发

1. 多组数据点进行标定，比较测试结果。（在获得一个一系列测试点组成的数据之后，随机选择点，判断，标定，判断精确度）
2. 拍照是否合格判断和整合
3. 根据新的数据类型重新设计程序：标定只会把机器人坐标和相机图像坐标储存到data中，其他工作都由另外读取的数据格式进行判断。



#### 数据读取

1. 一个data txt文件中包括H，p_camera, p_robot
2. 遍历，找到data开头的文件夹，储存路径，加上data.txt后缀作为data路径储存
3. 设计一个可以按照关键字读取的函数（从关键字开始读，读到空行为止），对于格式不规范问题暂时使用条件判断处理
4. 后续开发：
   1. 数据处理：比较变换矩阵H的误差：R、T标准运动学矩阵的误差计算
   2. 新的数据类型、后期程序（多点分析）：data point数据类型，储存robot pose和camera pose对应点
   3. 四组数据交叉标定交叉验证



功能：

1. 自动判断照片是否合格
2. 8点标定，16点标定



测试/评估：

1. 走棋盘格测试定位误差（程序开发部分已经完成），具体精度：画点，人工测量。
2. 利用相机视野不同区域标定，比较标定结果
3. 相机视野同一区域不同标定点，比较标定结果的误差
4. 使用不同数量的点标定，比较标定结果（8组，16组点，超定方程SVD解）



开发：

1. 功能；
   1. 通用
      1. 读取数据文件
      2. 将数据文件储存为point pair的格式
      3. 将给定point_pair_list（长度不一定）转homogeneous coordinate
   2. 合格性判断
      1. 法向向量判断
      2. 拟合半径判断
   3. 4点、8点、16点标定
      1. 4点标定：
         1. 随机选择4点合成pp_list
         2. valid check
         3. 输入p_robot, p_camera输出H
      2. 8点/16点标定
         1. 随机选择给定点数pp_list
         2. valid check
         3. SVD分解，求最小二乘解
   4. RANSAC标定
      1. 指定标定点数（指定算法）和循环次数
      2. 随机选择给定点数求解H
      3. evaluation
      4. 储存对应的num_list和evaluation结果
2. 测试/评估
   1. 数据端评估标定效果：给定一个H矩阵，对于每一个pp，计算用该H矩阵算得的误差得到一个误差list，求平均值
   2. 棋盘格误差估计：计算点数用的H要先将H矩阵转成rpy角度，用rpy再求一次R矩阵后构建新的H，用新的H来做转换。然后依次走棋盘格，用直尺测量误差，记录总误差，根据总点数计算。



任务：

* 开发：

  * 新的数据储存和交互模式
    * 采集数据阶段：储存机器人6DoF坐标, header - P_robot 6-DoF
      例子：
      p_robot 6-DoF index:1 
      ... （当前数据储存格式）
    * 标定阶段：以组为单位储存机器人位姿和对应的相机坐标，用当前时间作为index
      例子：
      p_robot 3-DoF index: xxxxx (current_time)
      [x,y,z]
      p_camera 3-DoF index: xxxxx (和上面一样)
      [x',y',z']
    * 数据写入程序：
      * 数据类型：向量，矩阵
      * 输入：数据，数据类型，index
  * 整合程序：
    * 数据获取
      * 生成路径
      * 移动及采集
      * 获取圆心并储存
    * 数据处理：标定



  其他功能开发：

  * 数据获取：
    * 移动及采集
      * 定点模式开发
      * 随机路径随机性及可靠性验证
    * 获取圆心
      * 检测是否有效，无效则删除





