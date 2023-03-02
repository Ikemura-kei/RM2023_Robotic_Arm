#include "ros/ros.h"
#include <arm_moveit/CMDInfo.h>
using namespace std;

int main (int argc, char **argv)
{
    ros::init (argc, argv , "fake_cmd_node");// node name


    ros::NodeHandle nh_; // variable n use just like "this" variable

    ros::Publisher cmd_pub = nh_.advertise<arm_moveit::CMDInfo> ("cmd_pub" , 10); 

    ros::Rate loop_rate(1);

    double count =0;
    while (ros::ok())
    {
        arm_moveit::CMDInfo cmd_info;
        cmd_info.header.stamp = ros::Time::now();
        cmd_info.name = "bocchi";
        cmd_info.mode = 0;

        cmd_info.positions ={0.262539,-0.00151127+count,0.198343};
        cmd_info.orientation={0,0,0,1};

        cmd_pub.publish(cmd_info);
        ros::spinOnce();
        loop_rate.sleep();
        count+=0.002;
    }

    return 0;


}