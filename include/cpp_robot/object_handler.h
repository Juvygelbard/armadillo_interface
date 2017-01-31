#ifndef OBJECT_HANDLER_H_
#define OBJECT_HANDLER_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

class ObjectHandler{
    private:
        double _stop_dist;
        ros::NodeHandle _nh;
        ros::ServiceClient _uc;
        std::vector<std::string> *_tracked; // ids of object which we are tracking
        moveit::planning_interface::PlanningSceneInterface *_ps_interface;
        tf::TransformListener *_tfl;
        void get_true_orientation(geometry_msgs::Pose &pose); // to be later subtitutd by the vision module

        void set_uc(bool val); // set state of update_collision_service

    public:
        ObjectHandler(double stop_dist=0.55);
        void add_tracked_object(std::string name);
        void remove_tracked_object(const std::string &name);
        geometry_msgs::Pose *get_object_pose(const std::string &name);

        ~ObjectHandler();
};

#endif