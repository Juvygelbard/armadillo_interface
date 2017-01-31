#ifndef DRIVER_INTERFACE_H_
#define DRIVER_INTERFACE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/costmap_model.h>

#include <geometry_msgs/Pose.h>

class DriverInterface{
    public:
        typedef move_base_msgs::MoveBaseGoal DIGoal;

    private:
        static const int NUM_OF_SAMPLING_POINTS = 32;

        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef void (*CallbackBool)(bool success);
    
        bool _ready;
        MBClient *_mb;
        tf::TransformListener *_tfl;
        costmap_2d::Costmap2DROS *_cm_interface;
        base_local_planner::CostmapModel *_cm_model;

        DIGoal xw_to_digoal(double x, double w);
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

    public:
        DriverInterface();

        // drive relative to map
        bool drive_block(geometry_msgs::Pose &pose, double radius=0);
        void drive_no_block(geometry_msgs::Pose &object, double radius=0);
        void drive_no_block(const CallbackBool callback, geometry_msgs::Pose &object, double radius=0);

        // drive relative to robot
        // TODO: implemen using odometry
        bool drive_block(double x, double w, double vel=0.2);
        void drive_no_block(double x, double w, double vel=0.2);
        void drive_no_block(const CallbackBool callback, double x, double w, double vel=0.2);

        void stop();

        ~DriverInterface();
};

#endif