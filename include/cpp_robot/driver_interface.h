#ifndef DRIVER_INTERFACE_H_
#define DRIVER_INTERFACE_H_

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class DriverInterface{
    public:
        typedef move_base_msgs::MoveBaseGoal DIGoal;

    private:
        typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MBClient;
        typedef actionlib::SimpleClientGoalState GoalState;
        typedef void (*CallbackBool)(bool success);
    
        bool _ready;
        MBClient *_mb;

        void generic_done_callback(const CallbackBool f, const GoalState &state);
        DIGoal xw_to_digoal(double x, double w);

    public:
        DriverInterface();
        bool drive_block(const DIGoal &goal);
        bool drive_block(double x, double w);
        void drive_no_block(const DIGoal &goal);
        void drive_no_block(double x, double w);
        void drive_no_block(const CallbackBool callback, const DIGoal &goal);
        void drive_no_block(const CallbackBool callback, double x, double w);
        void stop();
        ~DriverInterface();
};

#endif