#include <memory>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <arm_moveit/CMDInfo.h>
using namespace std;

const double tau = 2 * M_PI;



class CMD_Interface{
    protected:
        const std::string PLANNING_GROUP;
        moveit::planning_interface::MoveGroupInterface move_group_interface;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        const moveit::core::JointModelGroup* joint_model_group;
        moveit_visual_tools::MoveItVisualTools visual_tools;
        ros::Subscriber CMD_sub;
        int mode;

    public:
        CMD_Interface(ros::NodeHandle *nh_, string group_name, string end_point):
            PLANNING_GROUP(group_name),move_group_interface(group_name),visual_tools(end_point)
        {
            joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            CMD_sub = nh_->subscribe("cmd_pub",10,&CMD_Interface::cmd_info_CB, this);
        }

        void print_basic_info ()
        {
            ROS_INFO_NAMED("cmd_interface", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
            ROS_INFO_NAMED("cmd_interface", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
            ROS_INFO_NAMED("cmd_interface", "Available Planning Groups:");
            std::copy(move_group_interface.getJointModelGroupNames().begin(),
                move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
            cout << endl; 
        }

        void print_current_pose()
        {
            geometry_msgs::PoseStamped pose_msg = move_group_interface.getCurrentPose();
            cout << "Current end point position: " <<endl; 
            cout << "x coordinate:  "<<pose_msg.pose.position.x <<endl;
            cout << "y coordinate:  "<<pose_msg.pose.position.y <<endl;
            cout << "z coordinate:  "<<pose_msg.pose.position.z <<endl;
            cout << "x orientation:  "<<pose_msg.pose.orientation.x <<endl;
            cout << "y orientation:  "<<pose_msg.pose.orientation.y <<endl;
            cout << "z orientation:  "<<pose_msg.pose.orientation.z <<endl;
            cout << "w orientation:  "<<pose_msg.pose.orientation.w <<endl;
        }

        void set_goal_pose(const boost::array<double, 3> positions,const boost::array<double, 4> orientation)
        {
            geometry_msgs::Pose goal_pose;
            goal_pose.orientation.x = orientation[0];
            goal_pose.orientation.y = orientation[1];
            goal_pose.orientation.z = orientation[2];
            goal_pose.orientation.w = orientation[3];
            goal_pose.position.x = positions[0];
            goal_pose.position.y = positions[1];
            goal_pose.position.z = positions[2];
            move_group_interface.setPoseTarget(goal_pose);
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            ROS_INFO_NAMED("cmd_interface", "plan %s", success ? "Success" : "FAILED");
            if (success) {move_group_interface.move();}
        }
        void cmd_info_CB(const arm_moveit::CMDInfo cmd_info)
        {
            this->mode = cmd_info.mode;
            if ( this->mode ==1){
                set_goal_pose(cmd_info.positions,cmd_info.orientation);
                print_current_pose();
            }
        }
        void fake_CB()
        {
            arm_moveit::CMDInfo cmd_info;
            cmd_info.positions={0.2,0,-0.1};
            cmd_info.orientation={0,0,0,1};
            set_goal_pose(cmd_info.positions,cmd_info.orientation);
            print_current_pose();
        }
        

};




int main (int argc, char **argv)
{
    ros::init (argc, argv , "armmoveit_node");// node name

    ros::NodeHandle nh_;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    // ros::Subscriber sub =node_handle.subscribe("cmd_info",10,cmd_info_CB);

    // CMD_Interface cmd_interface (&nh_,"bear_arm","end_point");
    CMD_Interface cmd_interface (&nh_,"bear_arm","Link_6");
    cmd_interface.print_basic_info();
    cmd_interface.print_current_pose();
    // static const std::string PLANNING_GROUP = "bear_arm";
    // moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    // const moveit::core::JointModelGroup* joint_model_group =
    // move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // // Visualization
    namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("end_point");



    // // Getting Basic Information
    // // We can print the name of the reference frame for this robot.
    // ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    // // We can also print the name of the end-effector link for this group.
    // ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    // // We can get a list of all the groups in the robot:
    // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    // std::copy(move_group_interface.getJointModelGroupNames().begin(),
    //             move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));
    
    // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start");
    



    // // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    // // std::vector<double> joint_group_positions;


    // move_group_interface.setMaxVelocityScalingFactor(1);
    // move_group_interface.setMaxAccelerationScalingFactor(1);

    // geometry_msgs::Pose target_pose1;
    
    // print_current_pose(&move_group_interface);


    // double count = 0;
    while (ros::ok())
    {
    //     moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState();
    //     current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
        // cmd_interface.print_current_pose();

    //     target_pose1.orientation.w = 1.0;
    //     target_pose1.position.x = 0.3 ;
    //     target_pose1.position.y =0;
    //     target_pose1.position.z =-0.245+count;
    //     move_group_interface.setPoseTarget(target_pose1);
        
    //     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //     bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //     ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space go al) %s", success ? "" : "FAILED");

    //     move_group_interface.move();
    //     count+=0.005;
        // cmd_interface.fake_CB();
        // (ros::Rate(1)).sleep();
    }
    
    ros::shutdown();
    return 0;
}
