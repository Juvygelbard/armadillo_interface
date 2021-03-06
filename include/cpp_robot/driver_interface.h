#ifndef DRIVER_INTERFACE_H_
#define DRIVER_INTERFACE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>
#include <actionlib/server/simple_action_server.h>
#include <cpp_robot/SimpleDriverAction.h>
#include <boost/atomic.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

class SimpleDriverServer{
    private:
        ros::NodeHandle *_nh;
        ros::CallbackQueue *_cbq;
        ros::Publisher _pub;
        tf::TransformListener *_tf;
        actionlib::SimpleActionServer<cpp_robot::SimpleDriverAction> *_sas;
        boost::atomic<bool> _active;
        
        void callback(const cpp_robot::SimpleDriverGoalConstPtr &goal);

    public:
        SimpleDriverServer();
        void stop_server();
        ~SimpleDriverServer();
};

class DriverInterface{
     private:
        static const int NUM_OF_SAMPLING_POINTS = 32;
        typedef move_base_msgs::MoveBaseGoal DIGoal; // TODO: change to something else
        typedef cpp_robot::SimpleDriverGoal SDGoal;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;
        typedef actionlib::SimpleActionClient<cpp_robot::SimpleDriverAction> SDClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef void (*CallbackBool)(bool success);
    
        boost::atomic<bool> _ready;
        MBClient *_mb;
        SDClient *_sd;
        tf::TransformListener *_tfl;
        costmap_2d::Costmap2DROS *_cm_interface;
        base_local_planner::CostmapModel *_cm_model;

        // used to wrap the user's callback function'
        void generic_done_callback(const CallbackBool f, const GoalState &state);
        // returns true if point is blocked by some object, i.e. not reachable
        bool pose_blocked(const geometry_msgs::Pose &point);
        // returns true if point1 is closer to base then point2
        static bool point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2);
        // returns closest available point in radius
        geometry_msgs::Pose *get_best_pose_in_rad(const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius);
        // build DIGoal and handle radius
        DIGoal *build_digoal(geometry_msgs::Pose &object, double radius);
        // runs the simple-driver server on a seperate thread
        void start_sd_server();


    public:
        DriverInterface();

        // drive relative to map
        bool drive_block(geometry_msgs::Pose &pose, double radius=0);
        void drive_no_block(geometry_msgs::Pose &object, double radius=0);
        void drive_no_block(const CallbackBool callback, geometry_msgs::Pose &object, double radius=0);

        // drive relative to robot
        // TODO: fix odometry
        bool drive_block(double dist, double z, double vel=0.2);
        void drive_no_block(double dist, double z, double vel=0.2);
        void drive_no_block(const CallbackBool callback, double dist, double z, double vel);

        void stop();

        ~DriverInterface();
};

#endif