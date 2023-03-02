// recommend to do object orientation
#include <ros/ros.h>
#include "std_msgs/String.h"
// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
//topic
#include <arm_moveit/CMDInfo.h>
#include <trajectory_test/GoalInfo.h>

using namespace std;


class CMD_IK{
  protected:
    ros::Publisher goal_pub;
    ros::Subscriber CMD_sub;
    robot_model_loader::RobotModelLoader robot_model_loader;
    const moveit::core::RobotModelPtr& kinematic_model;
    moveit::core::RobotStatePtr kinematic_state;
    const moveit::core::JointModelGroup* joint_model_group;
    const vector<std::string>& joint_names;
    vector<double> joint_values;
		trajectory_test::GoalInfo goal_info;

  public:
    //constructor init robot model + kinematic solver/state + robot joint config
    CMD_IK(ros::NodeHandle *nh_, string group_name, string loader_name)
            :robot_model_loader(loader_name),
            kinematic_model(robot_model_loader.getModel()),
            joint_model_group(kinematic_model->getJointModelGroup(group_name)),
            joint_names (joint_model_group->getVariableNames()),
            kinematic_state (new moveit::core::RobotState(kinematic_model))
    {
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      CMD_sub = nh_->subscribe("cmd_pub",10,&CMD_IK::cmd_info_CB, this);
      goal_pub = nh_->advertise<trajectory_test::GoalInfo> ("goal_pub" , 10);
      goal_info.name = "bear_controller";

      //print basic info
      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str()); 
      print_joint_state_info();
      print_psoition_info();
    }   

    void goal_info_publish(){
      goal_info.Joint_1.resize(1);
			goal_info.Joint_2.resize(1);
			goal_info.Joint_3.resize(1);
			goal_info.Joint_4.resize(1);
			goal_info.Joint_5.resize(1);
			goal_info.Joint_6.resize(1);
			goal_info.time.resize(1);
			goal_info.header.stamp = ros::Time::now();
      goal_info.Joint_1[0] = joint_values[0];
      goal_info.Joint_2[0] = joint_values[1];
      goal_info.Joint_3[0] = joint_values[2];
      goal_info.Joint_4[0] = joint_values[3];
      goal_info.Joint_5[0] = joint_values[4];
      goal_info.Joint_6[0] = joint_values[5];
      goal_info.time[0]=0.1;
      goal_pub.publish(goal_info);
    }
    
    //foward Kinematics: print out current joint info
    void print_joint_state_info (){     
      kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
      for (std::size_t i = 0; i < joint_names.size(); ++i)
      {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
      }
    }

    //foeward Kinematics: set joint for the robot
    void set_joint (std::vector<double> joint_values){
      kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
      ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
    }

    //inverse Kinematics: print out current position and orientation info
    void print_psoition_info (){ 
      const Eigen::Isometry3d& new_state = kinematic_state->getGlobalLinkTransform("Link_6");
      ROS_INFO_STREAM("end_effector: \n" << new_state.matrix() << "\n");
    }

    //inverse Kinematics: set ik for the robot
    void set_IK (const boost::array<double, 3> positions,const boost::array<double, 4> orientation){

      geometry_msgs::Pose goal_pose;
      goal_pose.orientation.x = orientation[0];
      goal_pose.orientation.y = orientation[1];
      goal_pose.orientation.z = orientation[2];
      goal_pose.orientation.w = orientation[3];
      goal_pose.position.x = positions[0];
      goal_pose.position.y = positions[1];
      goal_pose.position.z = positions[2];
      double timeout = 0.1;
      bool found_ik = kinematic_state->setFromIK(joint_model_group, goal_pose, timeout);

      if (found_ik)
      {
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), (joint_values[i]));
          goal_info_publish();
        }
      }
      else
      {
        ROS_INFO("Did not find IK solution");
      }
      print_psoition_info();
    }

    // topic callback
    void cmd_info_CB (arm_moveit::CMDInfo cmd_info)
    {
      if (cmd_info.mode == 0){
        set_IK(cmd_info.positions,cmd_info.orientation);
      } 
    }

};
int main(int argc, char** argv)
{
  //node init
  ros::init(argc, argv, "armik_node");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  CMD_IK cmd_ik (&node_handle,"bear_arm","robot_description");

  // //init robot model (manipulator info)
  // robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  // static const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  // ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // //init kinematic solver/state 
  // static moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  // kinematic_state->setToDefaultValues();
  // const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("bear_arm");

  // //obtain robot joint config
  // static const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
  // std::vector<double> joint_values;
  // kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);






  while(ros::ok){

  }





  ros::shutdown();
  return 0;
}


