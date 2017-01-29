#include <ros/ros.h>

#include <cpp_robot/robot_fsm.h>

// RobotFSM Implementation

RobotFSM::RobotFSM():
    _nodes(new std::map<int, RobotFSMNode*>()),
    _start(0)
{

}

int RobotFSM::execute(){
    int id = _start;
    while(id && ros::ok()){
        if(id == 1)
            return id;
        ROS_INFO("executing node %i", id);
        id = _nodes->at(id)->execute();
    }

    if(!ros::ok())
        return 0;

    return id;
}

bool RobotFSM::run(){
    return execute();
}

void RobotFSM::set_start(int id){
    _start = id;
}

void RobotFSM::add_node(int id, RobotFSMNode *node){
    if(!_nodes->insert(std::pair<int, RobotFSMNode*>(id, node)).second)
        ROS_ERROR("node %i was already defined!", id);
}

RobotFSM::~RobotFSM(){
    delete _nodes;
}

// FuncFSMNode Implementation

FuncFSMNode::FuncFSMNode(FuncFSMNode::ExecuteFunc func):
    _func(func)
{}

int FuncFSMNode::execute(){
    return _func();
}

FuncFSMNode::~FuncFSMNode(){}