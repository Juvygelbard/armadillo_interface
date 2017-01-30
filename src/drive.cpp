#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include <cpp_robot/driver_interface.h>
#include <cpp_robot/object_handler.h>
#include <cpp_robot/arm_interface.h>
#include <cpp_robot/head_interface.h>
#include <cpp_robot/robot_fsm.h>

void ending(bool success){
	if(success)
		ROS_INFO("Done Moving!");
	else
		ROS_INFO("Failed Moving!");
}

int a(){
	HeadInterface hi;
	DriverInterface di;
	hi.point_head_no_block(0, 0, 0);
	di.drive_block(1.0, 1.0);
	return 3;
}

int b(){
	ObjectHandler hi;
	geometry_msgs::Pose *p = hi.get_object_pose("can");
	if(p)
		return 4;
	else{
		ROS_INFO("Can't find");
		return 5;
	}
}

int c(){
	ObjectHandler oh;
	HeadInterface hi;
	geometry_msgs::Pose *p = oh.get_object_pose("can");
	hi.point_head_block(p->position.x, p->position.y, p->position.z);
	return 1;
}

int d(){
	DriverInterface di;
	di.drive_block(-1.0, -1.0);
	return 0;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "navigate");
	ros::NodeHandle nh;

	DriverInterface di;
	ObjectHandler oh;
	ArmInterface ai;
	HeadInterface hi;
	ros::Duration(1).sleep();

	// DriverInterface::DIGoal *g = oh.get_object_digoal("can");
	geometry_msgs::Pose *p = oh.get_object_pose("can");
	if(p){
		ROS_INFO("Found can!");
		ai.close_gripper_no_block();
		di.drive_block(*p, 0.55);
		ROS_INFO("Reaching can...");
		ai.push_button(*p);
		ROS_INFO("Done!");
	}
	else{
		ROS_INFO("Can't find can!");
	}

	// if(g){
	// 	ROS_INFO("Found can!");
	// 	di.drive_no_block(&ending, *g);
	// }
	// else{
	// 	ROS_INFO("Can't find can!");
	// // }

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
	
	ros::spin();

	// // delete g;

	// delete a_node;
	// delete b_node;
	// delete c_node;

	return 0;
}