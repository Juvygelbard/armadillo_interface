#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include <cpp_robot/driver_interface.h>
#include <cpp_robot/object_handler.h>
#include <cpp_robot/arm_interface.h>
#include <cpp_robot/head_interface.h>
#include <cpp_robot/robot_fsm.h>

// void ending(bool success){
// 	if(success)
// 		ROS_INFO("Done Moving!");
// 	else
// 		ROS_INFO("Failed Moving!");
// }

// int a(){
// 	HeadInterface hi;
// 	DriverInterface di;
// 	hi.point_head_no_block(0, 0, 0);
// 	di.drive_block(1.0, 1.0);
// 	return 3;
// }

// int b(){
// 	ObjectHandler hi;
// 	geometry_msgs::Pose *p = hi.get_object_pose("can");
// 	if(p)
// 		return 4;
// 	else{
// 		ROS_INFO("Can't find");
// 		return 5;
// 	}
// }

// int c(){
// 	ObjectHandler oh;
// 	HeadInterface hi;
// 	geometry_msgs::Pose *p = oh.get_object_pose("can");
// 	hi.point_head_block(p->position.x, p->position.y, p->position.z);
// 	return 1;
// }

// int d(){
// 	DriverInterface di;
// 	di.drive_block(-1.0, -1.0);
// 	return 0;
// }

int main(int argc, char **argv){
	ros::init(argc, argv, "navigate");
	ros::NodeHandle nh;

	DriverInterface di;
	// ObjectHandler oh;
	// ArmInterface ai;
	HeadInterface hi;
	ros::Duration(1).sleep();

	ROS_INFO("calling...");
	di.drive_block(1.0, 0.3);
	while(ros::ok()){
		hi.point_head_block(1.0, 1.0, 0.0);
		hi.point_head_block(1.0, -1.0, 0.0);
	}

	ros::spin();

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

	// FuncFSMNode *a_node = new FuncFSMNode(&a);
	// FuncFSMNode *b_node = new FuncFSMNode(&b);
	// FuncFSMNode *c_node = new FuncFSMNode(&c);
	// FuncFSMNode *d_node = new FuncFSMNode(&d);

	// RobotFSM fsm;
	// fsm.add_node(2, a_node);
	// fsm.add_node(3, b_node);
	// fsm.add_node(4, c_node);
	// fsm.add_node(5, d_node);
	// fsm.set_start(2);
	// ending(fsm.run());
	
	// ros::spin();

	// delete a_node;
	// delete b_node;
	// delete c_node;

	return 0;
}