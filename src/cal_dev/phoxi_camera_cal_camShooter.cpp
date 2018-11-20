# include "phoxi_camera_cal_lib.h"
using namespace std;
using namespace ros;

bool phoxi_connect(const string& cam_id)
{
    ros::NodeHandle n;
    ros::ServiceClient ConnectCamera_client = n.serviceClient<phoxi_camera::ConnectCamera>("phoxi_camera/connect_camera");
    phoxi_camera::ConnectCamera ConnectCamera_srv;
    ConnectCamera_srv.request.name = cam_id;
    if (ConnectCamera_client.call(ConnectCamera_srv)) { ROS_INFO("Camera Conncted"); }
    else 
    {
        ROS_ERROR("Cammera Not Connected");
        cout << RD("Filed with Message:") << ConnectCamera_srv.response.message << endl;
    }
    return ConnectCamera_srv.response.success;
}

bool get_frame(const int id)
{
    ros::NodeHandle n;
    ros::ServiceClient GetFrame_client = n.serviceClient<phoxi_camera::GetFrame>("phoxi_camera/get_frame");
    phoxi_camera::GetFrame GetFrame_srv;
    GetFrame_srv.request.in = id;
    if (GetFrame_client.call(GetFrame_srv)) { ROS_INFO("Get Frame Success");}
    else                       
    { 
        ROS_ERROR("Get Frame Failed"); 
        cout << RD(Failed With Message:) << GetFrame_srv.response.message << endl;
    }
    return GetFrame_srv.response.success;
}

void image_capturing (const string& cam_id)
{
    static int times = 0;
    times++;
    // if (times == 1)
    // {
    //     cout << GR(Initializing Photoneo Camera) << endl;
    //     ros::Duration(5).sleep();
    // }
    cout << GR([#INFO] Calling Time:) << times << endl;
    double start, end, duration;
    // start = clock();
    
    ros::NodeHandle n;
    // Connect to the camera on PhoxiControler
    bool connectCamera_success = phoxi_connect(cam_id);
    bool getFrame_success = get_frame(-1);
    if (connectCamera_success & getFrame_success)
        ROS_INFO("CamShooter Success");
} 

int main (int argc, char** argv)
{
    ros::init(argc, argv, "phoxi_cal_camShooter");
    int max_iteration;
    ros::param::get("~num_of_iteration", max_iteration);
    cout << GR([#INFO] Num Iteration:) << max_iteration << endl;
    for (int i = 0; i < max_iteration; i++)
        image_capturing("1711004");
}