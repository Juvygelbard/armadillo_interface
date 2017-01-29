#ifndef HEAD_INTERFACE_H_
#define HEAD_INTERFACE_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/PointHeadGoal.h>

class HeadInterface{
    private:
        typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef pr2_controllers_msgs::PointHeadGoal HIGoal;
        typedef void (*CallbackBool)(bool success);

        bool _ready;
        PointHeadClient *_hi;

        HIGoal xyz_to_goal(std::string ref_frame, double x, double y, double z, double vel);
        void generic_done_callback(const CallbackBool f, const GoalState &state);

    public:
        HeadInterface();
        bool point_head_block(double x, double y, double z, double vel=1.0);
        void point_head_no_block(double x, double y, double z, double vel=1.0);
        void point_head_no_block(const CallbackBool callback, double x, double y, double z, double vel=1.0);
        void stop();
        ~HeadInterface();
};

#endif