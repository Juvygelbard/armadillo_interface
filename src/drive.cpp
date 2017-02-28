#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include <cpp_robot/driver_interface.h>
#include <cpp_robot/object_handler.h>
#include <cpp_robot/arm_interface.h>
#include <cpp_robot/head_interface.h>
#include <cpp_robot/robot_fsm.h>

bool stop = false;

// drive node
int drive_forward(){
	DriverInterface di;
	di.drive_block(1.0, -0.3);
	return 3;
}

int look_around(){
	// HeadInterface hi;
	// while(ros::ok() && !stop){
	// 	hi.point_head_block(1.0, 1.0, 0.0);
	// 	hi.point_head_block(1.0, -1.0, 0.0);
	// }
	for(int i=0; i<20; i++){
		std::cout << i << std::endl;
		ros::Duration(1.0).sleep();
	}
	return 4;
}

int stop_all(){
	stop = true;
	return 1;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigate");
	ros::NodeHandle nh;

	// DriverInterface di;
	// ObjectHandler oh;
	// ArmInterface ai;
	// HeadInterface hi;
	// ros::Duration(1).sleep();

	// geometry_msgs::Pose *p = oh.get_object_pose("can");
	// if(p){
	// 	ROS_INFO("Found can!");
	// 	di.drive_block(*p, 0.55);
	// 	ROS_INFO("Reaching can...");
	// 	ai.pickup_block(*p, "can");
	// 	ROS_INFO("Victory pose...");
	// 	ai.move_arm_block(0.4, 0.0, 0.5);
	// 	ROS_INFO("Placing back...");
	// 	ai.place_block(*p, "can");
	// 	ROS_INFO("Done!");
	// }
	// else{
	// 	ROS_INFO("Can't find can!");
	// }

	FuncFSMNode drive_forward_node(&drive_forward);
	FuncFSMNode look_around_node(&look_around);
	FuncFSMNode stop_all_node(&stop_all);

	std::vector<FSMNode*> conj_nodes;
	conj_nodes.push_back(&drive_forward_node);
	conj_nodes.push_back(&look_around_node);
	
	ConjFSMNode drive_and_look_node(conj_nodes);

	RobotFSM fsm;
	fsm.add_node(2, &drive_and_look_node);
	fsm.add_node(3, &stop_all_node);
	fsm.set_start(2);
	ROS_INFO(fsm.run() ? "Success!" : "Fail!");

	ros::spin();
	return 0;
}