#ifndef ARM_INTERFACE_H_
#define ARM_INTERFACE_H_

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf/transform_listener.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <control_msgs/GripperCommandAction.h>


class ArmInterface{
    private:
        typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
        typedef void (*CallbackBool)(bool success);
        typedef actionlib::SimpleClientGoalState GoalState;

        moveit::planning_interface::MoveGroup *_mg;
        moveit::planning_interface::PlanningSceneInterface *_psi;
        ros::AsyncSpinner *_as;
        tf::TransformListener *_tf;
        GripperClient *_gc;

        geometry_msgs::PoseStamped xyz_rpy_to_ps(double x, double y, double z, double r_ori, double p_ori, double y_ori);
        void generic_done_callback(const CallbackBool f, const GoalState &state);

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
        void push_button(const geometry_msgs::Pose &button); // initil - upgrade to a constarints aware version.
        void pick(const geometry_msgs::Pose &object);
        ~ArmInterface();
};

#endif