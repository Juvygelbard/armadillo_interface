#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>

#include <cpp_robot/object_handler.h>

// TODO: build callback mechanism, fix point->move calc
// TODO: delete digoal methods, including _tfl.

ObjectHandler::ObjectHandler(double stop_dist): 
        _stop_dist(stop_dist),
        _nh(),
        _uc(_nh.serviceClient<std_srvs::SetBool>("update_collision_objects")),
        _tracked(new std::vector<std::string>()),
        _ps_interface(new moveit::planning_interface::PlanningSceneInterface),
        _tfl(new tf::TransformListener){
    ROS_INFO("Waiting for uc service...");
    _uc.waitForExistence();
    set_uc(true);
    ROS_INFO("Object handler ready.");
}

void ObjectHandler::set_uc(bool val){
    std_srvs::SetBool srv;
    srv.request.data=val;
    _uc.call(srv);
}

void ObjectHandler::add_tracked_object(std::string name){
    _tracked->push_back(name);
}

void ObjectHandler::remove_tracked_object(const std::string &name){
    for(std::vector<std::string>::iterator it = _tracked->begin(); it != _tracked->end(); ++it){
        if(*it == name){
            _tracked->erase(it);
            break;
        }
    }
}

void ObjectHandler::get_true_orientation(geometry_msgs::Pose &pose){
    
}

geometry_msgs::Pose *ObjectHandler::get_object_pose(const std::string &name){
    // get objects pose map
    std::vector<std::string> ids;
    ids.push_back(name);
    std::map<std::string, geometry_msgs::Pose> poses = _ps_interface->getObjectPoses(ids);

    // return null if map is empty
    if(poses.empty())
        return 0;

    // stop looking
    return new geometry_msgs::Pose(poses[name]);
}

ObjectHandler::~ObjectHandler(){
    set_uc(false);
    delete _tracked;
    delete _ps_interface;
    delete _tfl;
}

