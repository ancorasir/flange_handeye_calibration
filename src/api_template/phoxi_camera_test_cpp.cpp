# include "phoxi_camera_test_cpp.h"
using namespace std;
using namespace ros;

void depth_im_call_back(const sensor_msgs::Image::ConstPtr& msg)
{
    ROS_INFO("Message received in callback function!!!");
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "phoxi_api_runner");
    ros::NodeHandle n;
    ros::ServiceClient ConnectCamera_client = n.serviceClient<phoxi_camera::ConnectCamera>("phoxi_camera/connect_camera");
    phoxi_camera::ConnectCamera ConnectCamera_srv;
    ConnectCamera_srv.request.name = "1711004";
    ConnectCamera_client.call(ConnectCamera_srv);
    if (ConnectCamera_srv.response.success){cout<<"yes"<<endl;}
    else {cout << ConnectCamera_srv.response.message << endl;}

    ros::ServiceClient GetFrame_client = n.serviceClient<phoxi_camera::GetFrame>("phoxi_camera/get_frame");
    phoxi_camera::GetFrame GetFrame_srv;
    GetFrame_srv.request.in = -1;
    if (GetFrame_client.call(GetFrame_srv))
    {
        ROS_INFO("Get Image");
        cout << GetFrame_srv.response.message << endl;
        ros::Subscriber sub = n.subscribe("/phoxi_camera/depth_map",1000,depth_im_call_back);
        ros::spin();
    }
    else
    {
        ROS_INFO("Failed");
        return -1;
    }
    

    return 0;
} 