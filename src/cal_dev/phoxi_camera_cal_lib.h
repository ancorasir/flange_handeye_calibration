#ifndef POHXI_CAMERA_CAL_LIB
#define POHXI_CAMERA_CAL_LIB
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "phoxi_camera/ConnectCamera.h"
#include "phoxi_camera/SaveFrame.h"
#include "phoxi_camera/GetFrame.h"
#include "phoxi_camera/GetDeviceList.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/PointCloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>
// For image transportation
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// Macro Definition for coloring 
#define RD(str) "\033[31;5m"#str"\033[0m"
#define GR(str) "\033[32;5m"#str"\033[0m"

using namespace std;
using namespace ros;

void depth_im_call_back(const sensor_msgs::Image::ConstPtr& depth_im);
void pcl_call_back(const sensor_msgs::PointCloud2::ConstPtr& pcl_map);
bool phoxi_connect(const string& cam_id);
bool get_frame(const int id);

#endif