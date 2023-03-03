# RM2023_Robotic_Arm

two launch file include inside ra_bring_up pkg
1_  rag1.launch :  for real connect to real world robot
2_  rag1_fake.launch : stand alone fake control for testing

three interface topic:
1_  cmd_pub: with msg "CMDinfo.msg" inside arm_moveit pkg
             it is use for CMD control to moveit group detail plz look at the msg doc
2_  goal_pub: with msg "GoalInfo.msg" inside trajectory_test pkg
              it is use for trajectory information feedback detail plz look at the msg doc
3_  joint_states: with msg "JointState.msg" inside sensor_msgs pkg
                it is enable moveit group know the real world robot joint state and simlulate inside moveit. 





basic pkg/launch:
1_  arm_IK arm_ik.launch
    this node collect the cmd info and product a inverse kinematic solution for the robot without plan the trajectory and simulation.So only one position for each time. it is way faster then the normal moveit group interface.Each solution <0.1s Therefore, when cmd.mode = 0 i chose to use this node. It is a stand alone node. The result will directly pulish to the goal_pub. Note: it can only generate one solution as it is iteration solution.

2_  arm_moveit armmoveit_node
    this node collect the cmd info and being the interface between moveit group. Then calculate a full trajectory path information. it is much slower than the last one.  Each solution form 0.1s to 0.3s depend on the travel distance. Therefore, when cmd.mode = 1 i chose to use this node.

3_  trajectory_test bear_controller
    This node work with armmoveit_node and pulish the trajectory path to the goal_pub. it is using action servers to comunicate to the moveit group

4_  rag1_config demo.launch
    This node start the simulation and moveit group. Visluating the the robot states. 

fake cmd and joint states pulisher:
1_  fake_cmd fake_cmd_node
    Genterate fake cmd_info and pulish to the cmd_pub for testing.

2_  actual_joint_state actual_joint_state_node
    Genterate fake joint state info and pulish to the joint_states.(i know the naming of this project suck)

How to use:
1_  create a pkg communicate with the actual robot to obtain the cmd and joint state information.
2_  create a cmd and joint state msg variable. Can take a reference to fake_cmd_node and 
    actual_joint_state_node
3_  Pulish the cmd msg to cmd_pub topic and joint state information from the real robot to joint_states topic.
4_  Subcribe to the goal_pub topic to obtain the Trajectory info of each joint.
5_  put the node into the rag1.launch launch file.
6_  type roslaunch ra_bring_up rag1.launch in terminal to run the moveit group.
7_  Downstream the Trajectory info to the embedden systems.


catkin build cmd:
1_  To build all pkg: catkin build -j2
2_  To build one pkg: catkin build "pkg name" --no-deps

init robot state:
    The init state plz follow to the picture as at that point all the joint angle is zero. And at this point the inverse kinematic matrix of the end-effector to the base coordinate is following. (zero rotation ,the translation in the first three row of the last col)
    1                -6.95282e-310  2.67773e-312    0.262539
    6.95282e-310     1              4.65177e-310    -0.00151127
    -2.67773e-312    -4.65177e-310  1               0.198343
    0                0              0               1

Note: Rag1 is only three asix control and current config has up to 6 asix, In order to make it consistance on the joint states info. Plz directly copy the value of the last three asix of the goal_info into the joint_states info each time request for the goal_info. 
