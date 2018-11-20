# include "phoxi_camera_cal_lib.h"
using namespace std;
using namespace ros;
using namespace pcl;

void my_pause()
{
    string pause;
    cout << GR(Press Enter to Continue) << endl;
    cin >> pause;
    return;
}

void passThroughFilter(PointCloud<pcl::PointXYZ>::Ptr& cloud, const string field, 
                const int min, const int max, const bool isNegative)
{
    pcl::PassThrough<pcl::PointXYZ> pass_trhough_filter;
    pass_trhough_filter.setInputCloud(cloud);
    pass_trhough_filter.setFilterFieldName(field);
    pass_trhough_filter.setFilterLimits(min, max);
    pass_trhough_filter.setFilterLimitsNegative(isNegative);
    pass_trhough_filter.filter(*cloud);
    return;
}

void outLierFilter(PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(50);
    outlier_filter.setStddevMulThresh(1.0);
    outlier_filter.filter(*cloud);
    return;
}

void clustering(PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // TODO: Unfinished Codes Here 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

    ec.setClusterTolerance (0.5);
    ec.setMinClusterSize (2000);
    ec.setMaxClusterSize (1000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    return;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "phoxi_cal");
    string im_path;
    ros::param::get("~im_path",im_path);
    double passthrough_min;
    ros::param::get("~passthrough_min", passthrough_min);
    double passthrough_max;
    ros::param::get("~passthrough_max", passthrough_max);
    // cout <<GR(++++ Param) <<"min:" << passthrough_min << " max:" << passthrough_max << endl;
    // cout << "Im Path" << im_path << endl;
	pcl::PCDReader reader;
	pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	reader.read<pcl::PointXYZ>(im_path, *cloud);
	passThroughFilter(cloud, "z", passthrough_min, passthrough_max, true);
    cout << GR(Pass Through Filtering Done)<< endl;
    pcl::io::savePCDFile("/home/bionicdl/calibration_images/pass_filter.PCD",*cloud);
    outLierFilter(cloud);
    cout << GR(Out Lier Filtering Done)<< endl;
    pcl::io::savePCDFile("/home/bionicdl/calibration_images/out_lier_filter.PCD",*cloud);
	return 0;	
}
            
                
									
						

