#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

#include <cpp_robot/robot_fsm.h>

// RobotFSM Implementation

RobotFSM::RobotFSM():
    _nodes(new std::map<int, FSMNode*>()),
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

void RobotFSM::add_node(int id, FSMNode *node){
    if(!_nodes->insert(std::pair<int, FSMNode*>(id, node)).second)
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

// DisjFSMNode Implementation

DisjFSMNode::DisjFSMNode(const std::vector<FSMNode*> nodes):
    _nodes(nodes),
    _cv(),
    _done_mutex(),
    _done(false),
    _next(0)
{}

void DisjFSMNode::worker(FSMNode *node){
    int next = node->execute();
    _done_mutex.lock();
    if(!_done){
        _next = next;
        _done = true;
        _cv.notify_all();
    }
    _done_mutex.unlock();
}

int DisjFSMNode::post_execution(int next){
    return next;
}

int DisjFSMNode::execute(){
    _done_mutex.lock();
    _done = false;
    _done_mutex.unlock();

    boost::mutex m;
    boost::unique_lock<boost::mutex> l(m);
    
    // start threads
    for(std::vector<FSMNode*>::const_iterator it = _nodes.begin(); it != _nodes.end(); ++it){
        boost::thread(&DisjFSMNode::worker, this, *it);
    }

    _cv.wait(l);
    return post_execution(_next);
}

DisjFSMNode::~DisjFSMNode(){

}

// ConjFSMNode Implementation

ConjFSMNode::ConjFSMNode(std::vector<FSMNode*> nodes):
    _nodes(nodes),
    _next(nodes.size()),
    _counter(0),
    _c_mut(),
    _cv(){}

void ConjFSMNode::dec_counter(){
    _c_mut.lock();
    _counter--;
    _c_mut.unlock();
}

bool ConjFSMNode::is_done(){
    _c_mut.lock();
    bool ans = _counter == 0;
    _c_mut.unlock();
    return ans;
}

int ConjFSMNode::post_execution(std::vector<int> &next){
    if(next.empty())
        return 0;
    else
        return next[0];
}

void ConjFSMNode::worker(int id){
    _next[id] = _nodes[id]->execute();

    // decrement counter and notify main thread
    dec_counter();
    _cv.notify_all();
}

int ConjFSMNode::execute(){
    _counter = _nodes.size();
    boost::mutex m;
    boost::unique_lock<boost::mutex> l(m);

    // start all threads
    for(int i=0; i<_nodes.size(); i++){
        boost::thread(&ConjFSMNode::worker, this, i);
    }

    // wait for all threads to finish
    while(!is_done()){
        _cv.wait(l);
    }

    return post_execution(_next);
}

ConjFSMNode::~ConjFSMNode(){

}