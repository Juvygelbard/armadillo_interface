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

        void generic_done_callback(const CallbackBool f, const GoalState &state);
        // returns true if point is blocked by some object, i.e. not reachable
        bool pose_blocked(const geometry_msgs::Pose &point);
        // returns true if point1 is closer to base then point2
        static bool point_dist_comperator(const geometry_msgs::Pose &base, const geometry_msgs::Pose &point1, const geometry_msgs::Pose &point2);
        // returns closest available point in radius
        geometry_msgs::Pose *get_best_pose_in_rad(const geometry_msgs::Pose &robot, const geometry_msgs::Pose &object, double radius);        DIGoal xw_to_digoal(double x, double w);

    public:
        DriverInterface();
        bool drive_block(const DIGoal &goal);
        bool drive_block(geometry_msgs::Pose &pose, double radius=0);
        bool drive_block(double x, double w);
        void drive_no_block(const DIGoal &goal);
        void drive_no_block(double x, double w);
        void drive_no_block(const CallbackBool callback, const DIGoal &goal);
        void drive_no_block(const CallbackBool callback, double x, double w);
        void stop();
        ~DriverInterface();
};

#endif