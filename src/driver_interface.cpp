#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>

#include <cpp_robot/driver_interface.h>

DriverInterface::DriverInterface(): _ready(false), _mb(0){
    // init move_base client
    _mb = new MBClient("move_base", true);
    while(!_mb->waitForServer(ros::Duration(5.0)) && ros::ok()){
		ROS_INFO("Waiting for move_base server.");
	}
    if(ros::ok())
        _ready = true;
}

DriverInterface::DIGoal DriverInterface::xw_to_digoal(double x, double w){
    DIGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.orientation.w = w;
    return goal;
}

bool DriverInterface::drive_block(const DIGoal &goal){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }
	_mb->sendGoalAndWait(goal);

	return _mb->getState() == GoalState::SUCCEEDED;
}

bool DriverInterface::drive_block(double x, double w){
    DIGoal goal = xw_to_digoal(x, w);
    return drive_block(goal);
}

void DriverInterface::drive_no_block(const DIGoal &goal){
    if(!_ready)
        ROS_ERROR("DriverInterface is not ready!");
    else
	    _mb->sendGoal(goal);
}

void DriverInterface::drive_no_block(double x, double w){
    DIGoal goal = xw_to_digoal(x, w);
    drive_no_block(goal);
}

void DriverInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    if(state == GoalState::SUCCEEDED)
        f(true);
    else
        f(false);
}

void DriverInterface::drive_no_block(const CallbackBool callback, const DIGoal &goal){
    if(!_ready)
        ROS_ERROR("DriverInterface is not ready!");
    else{
        _mb->sendGoal(goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
    }
}

void DriverInterface::drive_no_block(const CallbackBool callback, double x, double w){
    DIGoal goal = xw_to_digoal(x, w);
    drive_no_block(callback, goal);
}


void DriverInterface::stop(){
    _mb->cancelAllGoals();
}

DriverInterface::~DriverInterface(){
    if(_mb){
        delete _mb;
    }
}
