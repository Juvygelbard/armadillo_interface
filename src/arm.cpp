#include <vector>

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "moveit_group_goal");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle nh;

    moveit::planning_interface::MoveGroup group("arm");

    // group.setPlannerId("RRTConnectkConfigDefault");
    // group.setPoseReferenceFrame("base_footprint");
    // group.setStartStateToCurrentState();

    ROS_INFO("Planning Reference Frame: %s", group.getPlanningFrame().c_str());
    ROS_INFO("End Effector Reference Frame: %s", group.getEndEffectorLink().c_str());

    geometry_msgs::PoseStamped target_pose;
    target_pose.header.frame_id = "base_footprint";
    target_pose.header.stamp = ros::Time::now() + ros::Duration(2.1);
    target_pose.pose.position.x = 0.5;
    target_pose.pose.position.y = 0.0;
    target_pose.pose.position.z = 0.7;
    target_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.0);

    group.setPoseTarget(target_pose);

    // std::vector<double> group_joint_vals;
    // robot_state::RobotStatePtr state = group.getCurrentState();
    // state->copyJointGroupPositions(state->getRobotModel()->getJointModelGroup(group.getName()), group_joint_vals);
    // group_joint_vals[0] = -1.0;

    // group.setJointValueTarget(group_joint_vals);

    moveit::planning_interface::MoveGroup::Plan plan;
    bool success = group.plan(plan);

    ROS_INFO("plan: %s", success?"SUCCESS":"FAILED");

    if(success){
        ROS_INFO("Moving...");
        group.move();
    }

    ros::spin();
    return 0;
}