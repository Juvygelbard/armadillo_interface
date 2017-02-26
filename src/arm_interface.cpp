#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <math.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/Plane.h>
#include <control_msgs/GripperCommandGoal.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <cpp_robot/arm_interface.h>

ArmInterface::ArmInterface():
    _ready(false),
    _mg(new moveit::planning_interface::MoveGroup("arm")),
    _psi(new moveit::planning_interface::PlanningSceneInterface()),
    _as(new ros::AsyncSpinner(1)),
    _tf(new tf::TransformListener),
    _gc(new GripperClient("/gripper_controller/gripper_cmd", true)),
    _puc(new PickupClient("pickup", true)),
    _plc(new PlaceClient("place", true))
{
    ros::Duration w(1.0);
    while (!(_gc->waitForServer() && _puc->waitForServer() && _plc->waitForServer()) && ros::ok()){
        ROS_INFO("Waiting for action servers...");
        w.sleep();
    }
}

geometry_msgs::PoseStamped ArmInterface::xyz_rpy_to_ps(double x, double y, double z, double r_ori, double p_ori, double y_ori){
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "base_footprint";
    goal.header.stamp = ros::Time::now();
    goal.pose.position.x = x;
    goal.pose.position.y = y;
    goal.pose.position.z = z;
    goal.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r_ori, p_ori, y_ori);
    return goal;
}

// TODO: add roll/pitch/yaw?
bool ArmInterface::move_arm_block(double x, double y, double z){
    // create goal
    geometry_msgs::PoseStamped goal = xyz_rpy_to_ps(x, y, z, 0.0, 0.0, 0.0);
    
    // send goal to movegroup, plan and execute
    _mg->setPoseTarget(goal);
    moveit::planning_interface::MoveGroup::Plan plan;

    // start async spinner for plan callback to return
    _as->start();
    if(_mg->plan(plan)){
        _as->stop();
        _mg->execute(plan);
        return true;
    }
    _as->stop();
    return false;
}

void ArmInterface::move_arm_no_block(double x, double y, double z){
    // create goal
    geometry_msgs::PoseStamped goal = xyz_rpy_to_ps(x, y, z, 0.0, 0.0, 0.0);
    
    // send goal to movegroup, plan and execute
    _mg->setPoseTarget(goal);
    moveit::planning_interface::MoveGroup::Plan plan;

    // start async spinner for plan callback to return
    _as->start();
    if(_mg->plan(plan)){
        _mg->asyncExecute(plan);
    }
    _as->stop();
}

void ArmInterface::push_button(const geometry_msgs::Pose &button){
    // get transform of map-to-arm
    tf::StampedTransform arm_tf;
    _tf->lookupTransform("base_link", "map", ros::Time(0), arm_tf);

    // apply transform to object loaction
    tf::Vector3 item_pos(button.position.x, button.position.y, button.position.z);
    tf::Quaternion item_ori(button.orientation.x, button.orientation.y, button.orientation.z, button.orientation.w);
    tf::Vector3 tf_pos = arm_tf * item_pos;
    tf::Quaternion tf_ori = arm_tf * item_ori;
    tf::Matrix3x3 mat(tf_ori);
    double r, p, y;
    mat.getRPY(r, p, y);

    // build targets and constraints
    geometry_msgs::PoseStamped push_pos = xyz_rpy_to_ps(tf_pos.x(), tf_pos.y(), tf_pos.z(), r, p, y);
    geometry_msgs::PoseStamped ready_pos = xyz_rpy_to_ps(tf_pos.x()-0.2, tf_pos.y(), tf_pos.z(), r, p, y);

    // add constaints
    double coef_a = tan(y-0.5*M_PI);
    double coef_d = -1*(tf_pos.x()+0.2) + tf_pos.y();

    shape_msgs::Plane wall_plane;
    wall_plane.coef[0] = coef_a;
    wall_plane.coef[1] = 0.0;
    wall_plane.coef[2] = 0.0;
    wall_plane.coef[3] = coef_d;

    moveit_msgs::CollisionObject wall;
    wall.header.frame_id = "base_link";
    wall.id = "button_wall";
    wall.planes.push_back(wall_plane);

    std::vector<moveit_msgs::CollisionObject> objects;
    objects.push_back(wall);
    // _psi->addCollisionObjects(objects);

    // move arm to target
    _mg->setPoseTarget(push_pos);
    moveit::planning_interface::MoveGroup::Plan plan;

    // start async spinner for plan callback to return
    _as->start();
    if(_mg->plan(plan)){
        _mg->asyncExecute(plan);
    }
    _as->stop();
}

// TODO: change reference frame for grasp. make grasp closer to robot.
moveit_msgs::PickupGoal *ArmInterface::build_pickup_goal(const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PickupGoal *goal = new moveit_msgs::PickupGoal;

    goal->target_name = object;
    goal->group_name = "arm";
    goal->end_effector = "eef";
    goal->allow_gripper_support_collision = false;
    goal->minimize_object_distance = false;

    ///
    goal->allowed_planning_time = 5.0;
    goal->planning_options.replan_delay = 2.0;
    goal->planning_options.planning_scene_diff.is_diff = true;
    goal->planning_options.planning_scene_diff.robot_state.is_diff = true;
    goal->planning_options.replan=true;
    goal->planning_options.replan_attempts=5;
    goal->planner_id = "RRTConnectkConfigDefault";
    ///

    moveit_msgs::Grasp g;
    // position of gripper before grasp
    g.pre_grasp_posture.joint_names.push_back("left_finger_joint");
    g.pre_grasp_posture.joint_names.push_back("right_finger_joint");
    g.pre_grasp_posture.points.resize(1);
    g.pre_grasp_posture.points[0].positions.resize(2);
    g.pre_grasp_posture.points[0].positions[0] = 0.14;
    
    // position of gripper during grasp (+ grasp force)
    g.grasp_posture.joint_names.push_back("left_finger_joint");
    g.grasp_posture.joint_names.push_back("right_finger_joint");
    g.grasp_posture.points.resize(1);
    g.grasp_posture.points[0].positions.resize(2);
    g.grasp_posture.points[0].positions[0] = 0.01;
    g.grasp_posture.points[0].effort.resize(2);
    g.grasp_posture.points[0].effort[0] = 0.4;

    // position of end-effector during grasp
    g.grasp_pose.header.frame_id = "base_link";
    tf::StampedTransform robot_tf;
    _tf->lookupTransform("base_link", "map", ros::Time(0), robot_tf);
    
    tf::Vector3 v_pos(pose.position.x, pose.position.y, pose.position.z);
    tf::Vector3 tf_pos = robot_tf * v_pos;
    tf::Quaternion tf_ori;
    // robot_tf.getBasis().getRotation(tf_ori);
    
    g.grasp_pose.pose.position.x = tf_pos.getX() - 0.02;
    g.grasp_pose.pose.position.y = tf_pos.getY();
    g.grasp_pose.pose.position.z = tf_pos.getZ();

    double yaw = atan2( tf_pos.getY(),  tf_pos.getX());
    tf_ori.setRPY(0.0, 0.0, yaw);
    
    g.grasp_pose.pose.orientation.x = tf_ori.getX();
    g.grasp_pose.pose.orientation.y = tf_ori.getY();
    g.grasp_pose.pose.orientation.z = tf_ori.getZ();
    g.grasp_pose.pose.orientation.w = tf_ori.getW();
/*
    g.grasp_pose.header.frame_id = "map";
    g.grasp_pose.pose.position.x = pose.position.x;
    g.grasp_pose.pose.position.y = pose.position.y;
    g.grasp_pose.pose.position.z = pose.position.z;

    // get robot yaw to be used in grasp pose
    tf::StampedTransform robot_tf;
    _tf->lookupTransform("map", "base_link", ros::Time(0), robot_tf);
    double r, p, y;
    robot_tf.getBasis().getEulerYPR(y, p, r);
    tf::Quaternion g_ori;
    g_ori.setRPY(0.0, 0.0, y);
    g.grasp_pose.pose.orientation.x = g_ori.getX();
    g.grasp_pose.pose.orientation.y = g_ori.getY();
    g.grasp_pose.pose.orientation.z = g_ori.getZ();
    g.grasp_pose.pose.orientation.w = g_ori.getW();
    */

    // location of end-effector before grasp
    g.pre_grasp_approach.direction.header.frame_id = "base_footprint";
    g.pre_grasp_approach.direction.vector.x = 1.0;
    g.pre_grasp_approach.min_distance = 0.1;
    g.pre_grasp_approach.desired_distance = 0.2;

    // location of end-effector after grasp
    g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
    g.post_grasp_retreat.direction.vector.z = 1.0;
    g.post_grasp_retreat.min_distance = 0.1;
    g.post_grasp_retreat.desired_distance = 0.2;

    g.max_contact_force = 1.0;

    goal->possible_grasps.push_back(g);
    return goal;
}

void ArmInterface::generic_done_callback(const CallbackBool f, const GoalState &state){
    f(state == GoalState::SUCCEEDED);
}

bool ArmInterface::pickup_block(const geometry_msgs::Pose &pose,  const std::string &object){
    moveit_msgs::PickupGoal *goal = build_pickup_goal(pose, object);
    _puc->sendGoalAndWait(*goal);
    delete goal;

    return _puc->getState() ==  GoalState::SUCCEEDED;
}

// TODO: CHECK IF POSSIBLE TO DELETE GOAL
void ArmInterface::pickup_no_block(const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PickupGoal *goal = build_pickup_goal(pose, object);
    _puc->sendGoal(*goal);
    // delete goal;
}

// TODO: CHECK IF POSSIBLE TO DELETE GOAL
void ArmInterface::pickup_no_block(CallbackBool callback, const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PickupGoal *goal = build_pickup_goal(pose, object);
    _puc->sendGoal(*goal, boost::bind(&ArmInterface::generic_done_callback, boost::ref(this), callback, _1));
    // delete goal;
}

moveit_msgs::PlaceGoal *ArmInterface::build_place_goal(const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PlaceGoal *goal = new moveit_msgs::PlaceGoal();
    goal->group_name = "arm";
    goal->attached_object_name = object;
    goal->place_eef = true;
    goal->allow_gripper_support_collision = false;
    
    ///
    goal->planner_id = "RRTConnectkConfigDefault";
    goal->allowed_planning_time = 5.0;
    goal->planning_options.replan = true;
    goal->planning_options.replan_attempts = 5;
    goal->planning_options.replan_delay = 2.0;
    goal->planning_options.planning_scene_diff.is_diff = true;
    goal->planning_options.planning_scene_diff.robot_state.is_diff = true;
    ///

    // building place location
    std::vector<moveit_msgs::PlaceLocation> pls;
    moveit_msgs::PlaceLocation pl;

    pl.place_pose.pose = pose;
    pl.place_pose.header.frame_id = "map";

    pl.pre_place_approach.direction.header.frame_id = "base_footprint";
    pl.pre_place_approach.direction.vector.z = -1.0;
    pl.pre_place_approach.min_distance = 0.1;
    pl.pre_place_approach.desired_distance = 0.2;

    pl.post_place_retreat.direction.header.frame_id = "gripper_link";
    pl.post_place_retreat.direction.vector.x = -1.0;
    pl.post_place_retreat.min_distance = 0.0;
    pl.post_place_retreat.desired_distance = 0.2;
    
    pls.push_back(pl);
    goal->place_locations = pls;

    return goal;
}

bool ArmInterface::place_block(const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PlaceGoal *goal = build_place_goal(pose, object);
    _plc->sendGoalAndWait(*goal);
    delete goal;

    return _plc->getState() ==  GoalState::SUCCEEDED;
}

void ArmInterface::place_no_block(const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PlaceGoal *goal = build_place_goal(pose, object);
    _plc->sendGoal(*goal);
    // delete goal;
}

// TODO: CHECK IF POSSIBLE TO DELETE GOAL
void ArmInterface::place_no_block(CallbackBool callback, const geometry_msgs::Pose &pose, const std::string &object){
    moveit_msgs::PlaceGoal *goal = build_place_goal(pose, object);
    _plc->sendGoal(*goal, boost::bind(&ArmInterface::generic_done_callback, boost::ref(this), callback, _1));
    // delete goal;
}

bool ArmInterface::set_gripper_block(double position, double force){
    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gc->sendGoalAndWait(goal);
}

void ArmInterface::set_gripper_no_block(double position, double force){
    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gc->sendGoal(goal);
}

void ArmInterface::set_gripper_no_block(CallbackBool callback, double position, double force){
    control_msgs::GripperCommandGoal goal;
    goal.command.position = position;
    goal.command.max_effort = force;
    _gc->sendGoal(goal, boost::bind(&ArmInterface::generic_done_callback, boost::ref(this), callback, _1));
}

bool ArmInterface::open_gripper_block(double force){
    return set_gripper_block(OPEN_GRIPPER, force);
}

void ArmInterface::open_gripper_no_block(double force){
    set_gripper_no_block(OPEN_GRIPPER, force);
}

void ArmInterface::open_gripper_no_block(CallbackBool callback, double force){
    set_gripper_no_block(callback, OPEN_GRIPPER, force);
}

bool ArmInterface::close_gripper_block(double force){
    return set_gripper_block(CLOSE_GRIPPER, force);
}

void ArmInterface::close_gripper_no_block(double force){
    set_gripper_no_block(CLOSE_GRIPPER, force);
}

void ArmInterface::close_gripper_no_block(CallbackBool callback, double force){
    set_gripper_no_block(callback, CLOSE_GRIPPER, force);
}

ArmInterface::~ArmInterface(){
    delete _mg;
    delete _psi;
    delete _as;
    delete _tf;
    delete _gc;
    delete _puc;
    delete _plc;
}