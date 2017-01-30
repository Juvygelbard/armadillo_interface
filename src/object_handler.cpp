#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/SetBool.h>

#include <cpp_robot/object_handler.h>
#include <cpp_robot/driver_interface.h>

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

DriverInterface::DIGoal *ObjectHandler::get_object_digoal(const std::string &name){
    // get raw pose of the object
    geometry_msgs::Pose *pose = get_object_pose(name);
    if(!pose)
        return 0;
    
    // get robot's current position relative to map
    tf::StampedTransform tf_base;
    _tfl->lookupTransform("map", "base_link", ros::Time(0), tf_base);

    // v_reorg = relative position of object
    tf::Vector3 v_base =  tf_base.getOrigin();
    tf::Vector3 v_obj(pose->position.x, pose->position.y, pose->position.z);
    tf::Vector3 v_reorg = v_obj - v_base;

    // calculate move params
    double m_yaw = atan2(v_reorg.y(), v_reorg.x());
    double m_dist = _stop_dist/sqrt(v_reorg.x()*v_reorg.x() + v_reorg.y()*v_reorg.y());
    tf::Vector3 v_dest = v_base + v_reorg*(1-m_dist);
    v_dest.setZ(0);

    // build actual move vals
    tf::Transform tf_dest;
    tf_dest.setOrigin(v_dest);

    tf::Quaternion tf_ori;
    tf_ori.setRPY(0.0, 0.0, m_yaw);
    tf_dest.setRotation(tf_ori);

    // build goal
    DriverInterface::DIGoal *goal = new DriverInterface::DIGoal();
    goal->target_pose.header.frame_id = "map";
    goal->target_pose.header.stamp = ros::Time::now();
    goal->target_pose.pose.position.x = tf_dest.getOrigin().x();
    goal->target_pose.pose.position.y = tf_dest.getOrigin().y();
    goal->target_pose.pose.position.z = 0;
    goal->target_pose.pose.orientation.x = tf_dest.getRotation().x();
    goal->target_pose.pose.orientation.y = tf_dest.getRotation().y();
    goal->target_pose.pose.orientation.z = tf_dest.getRotation().z();
    goal->target_pose.pose.orientation.w = tf_dest.getRotation().w();

    delete pose;
    return goal;
}

ObjectHandler::~ObjectHandler(){
    set_uc(false);
    delete _tracked;
    delete _ps_interface;
    delete _tfl;
}

