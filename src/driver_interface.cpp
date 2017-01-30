#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>

#include <cpp_robot/driver_interface.h>

DriverInterface::DriverInterface():
    _ready(false),
    _mb(0),
    _tfl(new tf::TransformListener),
    _cm_interface(new costmap_2d::Costmap2DROS("global_costmap", *_tfl)),
    _cm_model(new base_local_planner::CostmapModel(*_cm_interface->getCostmap())){
    // init move_base client
    _mb = new MBClient("move_base", true);
    while(!_mb->waitForServer(ros::Duration(5.0)) && ros::ok()){
		ROS_INFO("Waiting for move_base server.");
	}
    if(ros::ok())
        _ready = true;
}

bool DriverInterface::pose_blocked(const geometry_msgs::Pose &pose){
    // TODO: update to real inscribed_radius, circumscribed_radius
    double cost = _cm_model->footprintCost(pose.position, _cm_interface->getRobotFootprint(), 0.2, 0.3);
    return cost >= 0;
}

bool DriverInterface::point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2){
    double d1 = pow(base.position.x - point1.position.x, 2) + pow(base.position.y - point1.position.y, 2);
    double d2 = pow(base.position.x - point2.position.x, 2) + pow(base.position.y - point2.position.y, 2);
    return d1 <= d2;
}

geometry_msgs::Pose *DriverInterface::get_best_pose_in_rad(const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius){
    // create sampling points
    std::vector<geometry_msgs::Pose> poses;

    double theta_unit = (2 * M_PI)/NUM_OF_SAMPLING_POINTS;
    for(int i=0; i<NUM_OF_SAMPLING_POINTS; i++){
        geometry_msgs::Pose n_pose;
        
        // calc relative x, y, yaw
        double theta = theta_unit * i;
        double rel_x = radius * cos(theta);
        double rel_y = radius * sin(theta);
        double yaw = fmod(theta+M_PI, 2*M_PI);

        // transform to absolute vals
        n_pose.position.x = object.position.x + rel_x;
        n_pose.position.y = object.position.y + rel_y;
        n_pose.position.z = 0.0;

        tf::Quaternion ori;
        ori.setRPY(0.0, 0.0, yaw);

        n_pose.orientation.x = ori.getX();
        n_pose.orientation.y = ori.getY();
        n_pose.orientation.z = ori.getZ();
        n_pose.orientation.w = ori.getW();

        poses.push_back(n_pose);
    }

    // sort according to distance
    std::sort(poses.begin(), poses.end(),  boost::bind(&DriverInterface::point_dist_comperator, robot, _1, _2));

    // return first accessible point, or null if no such point
    for(std::vector<geometry_msgs::Pose>::iterator it=poses.begin(); it!=poses.end(); ++it){
        if(!pose_blocked(*it))
            return &(*it);
    }
    return 0;
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

bool DriverInterface::drive_block(geometry_msgs::Pose &object, double radius){
    // get robot position
    tf::StampedTransform robot_transform;
    _tfl->lookupTransform("map", "base_link", ros::Time(0), robot_transform);
    geometry_msgs::Pose robot;
    robot.position.x = robot_transform.getOrigin().x();
    robot.position.y = robot_transform.getOrigin().y();
    robot.position.z = 0.0;

    // if needed, get robot's new pose in radius from goal
    geometry_msgs::Pose *target;
    if(radius)
        target = get_best_pose_in_rad(robot, object, radius);
    else
        target = &object;

    // if no reachable goal, return false
    if(!target)
        return false;

    // build goal
    DIGoal *goal = new DIGoal();

    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();

    goal->target_pose.pose.position.x = target->position.x;
    goal->target_pose.pose.position.y = target->position.y;
    goal->target_pose.pose.position.z = 0.0;

    goal->target_pose.pose.orientation.x = target->orientation.x;
    goal->target_pose.pose.orientation.y = target->orientation.y;
    goal->target_pose.pose.orientation.z = target->orientation.z;
    goal->target_pose.pose.orientation.w = target->orientation.w;

    // send goal to move_base server
    return drive_block(*goal);
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
    if(_mb)
        delete _mb;
    if(_tfl)
        delete _tfl;
    if(_cm_interface)
        delete _cm_interface;
    if(_cm_model)
        delete _cm_model;
}
