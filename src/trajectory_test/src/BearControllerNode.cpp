// node purpose: collect Tranjectory information from moveit and upload to Topic "Goalinfo"
// method: Creating action server class in this node and communicate with moveit action client


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_test/GoalInfo.h>

using namespace std;


// action server class
class JointTrajectoryActionServer
{
	protected:
		ros::NodeHandle nh_;

		//init action server as "as_".
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
		actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction>::Result result_;
		std::string action_name_;

		//init topic "goal_hub" for publishing Tranjectory information
		//Custom msg GoalInfo
		ros::Publisher goal_pub = nh_.advertise<trajectory_test::GoalInfo> ("goal_pub" , 10);
		trajectory_test::GoalInfo goal_info;


	public:
		//action server constructor 
		JointTrajectoryActionServer(std::string name): as_(nh_, name, false), action_name_(name)
		{
			// register callback for goal
			as_.registerGoalCallback(boost::bind(&JointTrajectoryActionServer::goalCallback, this));
			as_.start();
			// init robot name for msg 
			goal_info.name = "bear_controller";
		}
		~JointTrajectoryActionServer(void){}	

		// when a trajectory command comes, this function will be called.
		void goalCallback()
		{
			// boost::shared_ptr<const control_msgs::FollowJointTrajectoryGoal> goal;
			// goal=as_.acceptNewGoal();
			// cout<<"trajectory point size:"<< goal->trajectory.points.size()<<endl;
			// // tell motion control hardware to execute
			// // do something
			// // when finished, return result
			// as_.setSucceeded(result_);
			
			// collect data from moveit client and save trajectory info to "trajectory"
			control_msgs::FollowJointTrajectoryGoal::_trajectory_type trajectory;
			trajectory_msgs::JointTrajectory::_points_type::iterator iter;
			trajectory = as_.acceptNewGoal()->trajectory;

			// copy infomation to msg goal_info and then publish
			goal_info.header.stamp = ros::Time::now();
			int counter=0;
			int array_size= trajectory.points.size();
			goal_info.Joint_1.resize(array_size);
			goal_info.Joint_2.resize(array_size);
			goal_info.Joint_3.resize(array_size);
			goal_info.Joint_4.resize(array_size);
			goal_info.Joint_5.resize(array_size);
			goal_info.Joint_6.resize(array_size);
			goal_info.time.resize(array_size);
			for(iter= trajectory.points.begin() ; iter!=trajectory.points.end(); iter++, counter++) 
			{
				
				goal_info.Joint_1[counter] = iter->positions[0];
				goal_info.Joint_2[counter] = iter->positions[1];
				goal_info.Joint_3[counter] = iter->positions[2];
				goal_info.Joint_4[counter] = iter->positions[3];
				goal_info.Joint_5[counter] = iter->positions[4];
				goal_info.Joint_6[counter] = iter->positions[5];
				goal_info.time[counter] = iter->time_from_start.toSec();
			}
			goal_pub.publish(goal_info);
			
		}

};

int main(int argc, char** argv)
{
	ros::init(argc,argv, "bear_controller");
	ROS_INFO_NAMED("action", "start");
	JointTrajectoryActionServer srv("bear_controller/follow_joint_trajectory");
	

	cout << "start running" <<endl;
	ros::spin();



	return 0;
}