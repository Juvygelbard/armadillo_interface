#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <actionlib/server/simple_action_server.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <cpp_robot/SimpleDriverAction.h>
#include <cpp_robot/SimpleDriverFeedback.h>

#include <cpp_robot/driver_interface.h>

SimpleDriverServer::SimpleDriverServer():
    _nh(new ros::NodeHandle()),
    _cbq(new ros::CallbackQueue()),
    _tf(new tf::TransformListener()),
    _sas(0),
    _active(true)
{
    _nh->setCallbackQueue(_cbq);
    _pub = _nh->advertise<geometry_msgs::Twist>("cmd_vel", 100);
    _sas = new actionlib::SimpleActionServer<cpp_robot::SimpleDriverAction>(*_nh, "simple_driver", boost::bind(&SimpleDriverServer::callback, boost::ref(this), _1), false);
    _tf->waitForTransform("base_footprint", "odom", ros::Time(0), ros::Duration(5.0));
    _sas->start();

    // spin local queue
    while(ros::ok() && _active){
        _cbq->callAvailable(ros::WallDuration(0));
    }  
}

void SimpleDriverServer::stop_server(){
    _active = false;
}

void SimpleDriverServer::callback(const cpp_robot::SimpleDriverGoalConstPtr &goal){
    tf::StampedTransform start_tf, curr_tf;
    cpp_robot::SimpleDriverFeedback feedback;
    double dist=0;

    // record starting position
    _tf->lookupTransform("base_footprint", "odom", ros::Time(0), start_tf);
    ros::Rate r(1);

    bool success = false;
    while(ros::ok() && !success && !_sas->isPreemptRequested()){
        // move
        _pub.publish(goal->cmd);
        r.sleep();

        // update distance traveled so-far
        _tf->lookupTransform("base_footprint", "odom", ros::Time(0), curr_tf);
        tf::Transform relative_tf = start_tf.inverse() * curr_tf;
        dist = relative_tf.getOrigin().length();
        feedback.distance = dist;
        _sas->publishFeedback(feedback);
        
        // check if done
        success = dist >= goal->distance;
    }
    
    if(success){
        cpp_robot::SimpleDriverResult result;
        result.distance = dist;
        _sas->setSucceeded(result);
    }
}

SimpleDriverServer::~SimpleDriverServer(){
    delete _sas;
    delete _nh;
    delete _cbq;
    delete _tf;
}

DriverInterface::DriverInterface():
    _ready(false),
    _mb(0),
    _tfl(new tf::TransformListener),
    _cm_interface(new costmap_2d::Costmap2DROS("global_costmap", *_tfl)),
    _cm_model(new base_local_planner::CostmapModel(*_cm_interface->getCostmap())),
    _sd(0){
    // start simple-driver server
    boost::thread sd_thread(boost::bind(&DriverInterface::start_sd_server, boost::ref(this)));

    // init move_base and simple-driver clients
    _mb = new MBClient("move_base", true);
    _sd = new SDClient("simple_driver", true);

    // wait for both servers to come-up
    ros::Duration w(1.0);
    while(!(_mb->waitForServer() && _sd->waitForServer()) && ros::ok()){
		ROS_INFO("Waiting for action servers...");
        w.sleep();
	}
    _ready = true;
}

void DriverInterface::start_sd_server(){
    ROS_INFO("starting simple-drive server...");
    SimpleDriverServer sd_server;
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
            return new geometry_msgs::Pose(*it);
    }
    return 0;
}

DriverInterface::DIGoal *DriverInterface::build_digoal(geometry_msgs::Pose &object, double radius){
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

    // if no reachable goal, return null
    if(!target)
        return 0;

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

    // delete target only if it was created by get_best_pose_in_rad(...)
    if(target != &object)
        delete target;
    
    return goal;
}

bool DriverInterface::drive_block(geometry_msgs::Pose &object, double radius){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }

    DIGoal *goal = build_digoal(object, radius);

    if(!goal)
        return false;

    _mb->sendGoalAndWait(*goal);

    delete goal;
	return _mb->getState() == GoalState::SUCCEEDED;
}


// NOTE: THERE IS A MEMORY LEAK HERE!
void DriverInterface::drive_no_block(geometry_msgs::Pose &object, double radius){
    if(!_ready)
        ROS_ERROR("DriverInterface is not ready!");
    else{
        DIGoal *goal = build_digoal(object, radius);
        _mb->sendGoal(*goal);
    }
}

void DriverInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

// NOTE: THERE IS A MEMORY LEAK HERE!
void DriverInterface::drive_no_block(const CallbackBool callback, geometry_msgs::Pose &object, double radius){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        callback(false);
    }
    else{
        DIGoal *goal = build_digoal(object, radius);
        _mb->sendGoal(*goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
    }
}

bool DriverInterface::drive_block(double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        return false;
    }
    SDGoal goal;
    goal.cmd.linear.x = vel;
    goal.cmd.angular.z = z;
    goal.distance = dist;
    _sd->sendGoalAndWait(goal);
    return _sd->getState() == GoalState::SUCCEEDED;
}

void DriverInterface::drive_no_block(double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
    }
    else{
        SDGoal goal;
        goal.cmd.linear.x = vel;
        goal.cmd.angular.z = z;
        goal.distance = dist;
        _sd->sendGoal(goal);
    }
}

void DriverInterface::drive_no_block(const CallbackBool callback, double dist, double z, double vel){
    if(!_ready){
        ROS_ERROR("DriverInterface is not ready!");
        callback(false);
    }
    else{
        SDGoal goal;
        goal.cmd.linear.x = vel;
        goal.cmd.angular.z = z;
        goal.distance = dist;
        _sd->sendGoal(goal, boost::bind(&DriverInterface::generic_done_callback, boost::ref(this), callback, _1));
    }
}


void DriverInterface::stop(){
    _mb->cancelAllGoals();
    _sd->cancelAllGoals();
}

DriverInterface::~DriverInterface(){
        delete _mb;
        delete _tfl;
        delete _cm_interface;
        delete _cm_model;
        delete _sd;
}
