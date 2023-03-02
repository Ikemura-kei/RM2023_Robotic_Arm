#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
using namespace std;
#include <trajectory_test/GoalInfo.h>




void joint_state_CB(const sensor_msgs::JointState& Joint_state_msg);
void fake_state_CB(const trajectory_test::GoalInfo goal_info );
sensor_msgs::JointState joint_state;
bool pub_confirm= false;



int main(int argc, char** argv)
{
	ros::init(argc,argv, "actual_joint_state_node");

	ros::NodeHandle node_handle;
    ros::Publisher joint_state_pub = node_handle.advertise<sensor_msgs::JointState> ("joint_states" , 10);
	// ros::Subscriber sub =node_handle.subscribe("actual_joint_state",10,joint_state_CB);
	ros::Subscriber sub =node_handle.subscribe("goal_pub",10,fake_state_CB);


    // define real joint state msg 
    joint_state.name.resize(6); 
    joint_state.position.resize(6);
    joint_state.name[0] ="Joint_1";
    joint_state.name[1] ="Joint_2";
    joint_state.name[2] ="Joint_3";
    joint_state.name[3] ="Joint_4";
    joint_state.name[4] ="Joint_5";
    joint_state.name[5] ="Joint_6";
    double count =0;
    
    // joint_state.header.stamp = ros::Time::now();
    // joint_state.position[0] = 0;
    // joint_state.position[1] = -0.303800;
    // joint_state.position[2] = 0.401673;
    // joint_state.position[3] = 0.097872;
    // joint_state.position[4] = 0;
    // joint_state.position[5] = 0;

    cout << "actual_joint_state_node start"<<endl;


	while (ros::ok())
	{
        count++;
		// if (count%10 ==0 ){ cout << count <<endl;}
        // receive actual world joint state
        ros::spinOnce();

        // publish to move_group/joint_states topic
        if (pub_confirm==true)
        {
            joint_state_pub.publish(joint_state);
            pub_confirm= false;
        }


        //testing joint state
        // joint_state.header.stamp = ros::Time::now();
        // joint_state.position[0] = 0;
        // joint_state.position[1] = -0.303800;
        // joint_state.position[2] = 0.401673;
        // joint_state.position[3] = 0.097872;
        // joint_state.position[4] = 0;
        // joint_state.position[5] = 0;
        // joint_state_pub.publish(joint_state);



		(ros::Rate(10)).sleep(); // defluat 10Hz
	}




	
	return 0;
}

void joint_state_CB(const sensor_msgs::JointState& Joint_state_msg)
{
    // updata joint state to moveit
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = Joint_state_msg.position[0];
    joint_state.position[1] = Joint_state_msg.position[1];
    joint_state.position[2] = Joint_state_msg.position[2];
    joint_state.position[3] = Joint_state_msg.position[3];
    joint_state.position[4] = Joint_state_msg.position[4];
    joint_state.position[5] = Joint_state_msg.position[5];
    // pub_confirm= true;
}


void fake_state_CB(const trajectory_test::GoalInfo goal_info )
{
    joint_state.header.stamp = ros::Time::now();
    joint_state.position[0] = goal_info.Joint_1[0];
    joint_state.position[1] = goal_info.Joint_2[0];
    joint_state.position[2] = goal_info.Joint_3[0];
    joint_state.position[3] = goal_info.Joint_4[0];
    joint_state.position[4] = goal_info.Joint_5[0];
    joint_state.position[5] = goal_info.Joint_6[0];
    pub_confirm= true;

}  
