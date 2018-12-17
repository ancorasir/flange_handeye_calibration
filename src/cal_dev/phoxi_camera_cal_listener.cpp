# include "phoxi_camera_cal_lib.h"
using namespace std;
using namespace ros;
using namespace pcl;
using namespace cv;



void depth_im_call_back(const sensor_msgs::Image::ConstPtr& depth_im)
{
    ROS_INFO("Depth map received in callback function!!!");
    return;
}

void pcl_call_back(const sensor_msgs::PointCloud2::ConstPtr& pcl_map)
{
    static int count = 0;
    count++;
    ROS_INFO("Point cloud map received in callback function!!!");
    string im_path;
    ros::param::get("~image_path", im_path);
    string im_path_ply = im_path + ".ply";
    string im_path_pcd = im_path + ".PCD";
    cout<< GR([#INFO] Image Path: ) << im_path << endl;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl_conversions::toPCL(*pcl_map, pcl_pc);
    pcl::fromPCLPointCloud2(pcl_pc, cloud);
    pcl::io::savePCDFile(im_path_pcd,cloud);
    pcl::io::savePLYFile(im_path_ply,cloud);
    return;
}


void rgb_call_back(const sensor_msgs::Image::ConstPtr& texture_map)
{
    ROS_INFO("encoding:[%s]",texture_map->encoding.c_str());
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(texture_map,  sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception:%s",e.what());
        return;
    }
    string im_path;
    ros::param::get("~image_path", im_path);
    string im_path_origin = im_path + "origin" + ".jpg";
    cv::imwrite(im_path_origin, cv_ptr->image);
}

// void passthrough(pcl::PointCloud)

int main(int argc,char** argv)
{
    ros::init(argc, argv, "phoxi_cal_listener");
    ros::NodeHandle n;
    ros::Subscriber sub_depthMap = n.subscribe("/phoxi_camera/depth_map", 5, depth_im_call_back);
    ros::Subscriber sub_pointCloud = n.subscribe("/phoxi_camera/pointcloud", 5, pcl_call_back);
    ros::Subscriber sub_texture = n.subscribe("/phoxi_camera/texture", 5, rgb_call_back);
    ros::spin();
}