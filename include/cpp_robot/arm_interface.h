#ifndef ARM_INTERFACE_H_
#define ARM_INTERFACE_H_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/atomic.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <control_msgs/GripperCommandAction.h>
#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PlaceAction.h>

class ArmInterface{
    private:
        typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
        typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickupClient;
        typedef actionlib::SimpleActionClient<moveit_msgs::PlaceAction> PlaceClient;
        typedef void (*CallbackBool)(bool success);
        typedef actionlib::SimpleClientGoalState GoalState;

        boost::atomic<bool> _ready;
        moveit::planning_interface::MoveGroup *_mg;
        moveit::planning_interface::PlanningSceneInterface *_psi;
        ros::AsyncSpinner *_as;
        tf::TransformListener *_tf;
        GripperClient *_gc;
        PickupClient *_puc;
        PlaceClient *_plc;

        geometry_msgs::PoseStamped xyz_rpy_to_ps(double x, double y, double z, double r_ori, double p_ori, double y_ori);
        void generic_done_callback(const CallbackBool f, const GoalState &state);
        moveit_msgs::PickupGoal *build_pickup_goal(const geometry_msgs::Pose &pose, const std::string &object, double d_x=0, double d_y=0, double d_z=0, double d_Y=0);
        moveit_msgs::PlaceGoal *build_place_goal(const geometry_msgs::Pose &pose, const std::string &object);


    public:
        static const double OPEN_GRIPPER = 0.2;
        static const double CLOSE_GRIPPER = 0.01;

        ArmInterface();

        // simple arm manipulation interface
        bool move_arm_block(double x, double y, double z);
        void move_arm_no_block(double x, double y, double z);
        // TODO: Implament a non-blocking method with cb.

        // gripper interface
        bool set_gripper_block(double position, double force=0.2);
        void set_gripper_no_block(double position, double force=0.2);
        void set_gripper_no_block(CallbackBool callback, double position, double force=0.2);
        bool open_gripper_block(double force=0.2);
        void open_gripper_no_block(double force=0.2);
        void open_gripper_no_block(CallbackBool callback, double force=0.2);
        bool close_gripper_block(double forec=0.2);
        void close_gripper_no_block(double forec=0.2);
        void close_gripper_no_block(CallbackBool callback, double forec=0.2);

        // complex arm actions interface
        void push_button(const geometry_msgs::Pose &button); // initial - upgrade to a constarints aware version.

        bool pickup_block(const geometry_msgs::Pose &pose, const std::string &object);
        void pickup_no_block(const geometry_msgs::Pose &pose, const std::string &object);
        void pickup_no_block(CallbackBool callback, const geometry_msgs::Pose &pose, const std::string &object);

        bool place_block(const geometry_msgs::Pose &pose, const std::string &object);
        void place_no_block(const geometry_msgs::Pose &pose, const std::string &object);
        void place_no_block(CallbackBool callback, const geometry_msgs::Pose &pose, const std::string &object);

        ~ArmInterface();
};

#endif