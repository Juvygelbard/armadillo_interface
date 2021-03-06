#ifndef ROBOT_FSM_H_
#define ROBOT_FSM_H_

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/condition_variable.hpp>

class FSMNode{
    public:
        virtual int execute()=0;
};

class RobotFSM: public FSMNode{
    private:
        std::map<int, FSMNode*> *_nodes;
        int _start;

    public:
        RobotFSM();
        int execute();
        bool run();
        void set_start(int id);
        void add_node(int id, FSMNode *node);
        ~RobotFSM();
};

class FuncFSMNode: public FSMNode{     
    public:
        typedef int (*ExecuteFunc)();

        FuncFSMNode(ExecuteFunc func);
        int execute();
        ~FuncFSMNode();
    
    private:
        ExecuteFunc _func;
};

// TODO: implement conjuction-node, disjunction-node, supervised-node (inherite from disj-node)

// rescives a vector of nodes, returns once the first node finishes
class DisjFSMNode: public FSMNode{
    private:
        const std::vector<FSMNode*> _nodes;
        boost::condition_variable _cv;
        boost::mutex _done_mutex;
        bool _done;
        int _next;

        void worker(FSMNode *node);
        // to be overriden
        virtual int post_execution(int next);

    public:
        DisjFSMNode(const std::vector<FSMNode*> nodes);
        int execute();
        ~DisjFSMNode();
};

class ConjFSMNode: public FSMNode{
    private:
        const std::vector<FSMNode*> _nodes;
        std::vector<int> _next;
        int _counter;
        boost::mutex _c_mut;
        boost::condition_variable _cv;

        void dec_counter();
        bool is_done();
        void worker(int id);
        virtual int post_execution(std::vector<int> &next);

    public:
        ConjFSMNode(std::vector<FSMNode*> nodes);
        int execute();
        ~ConjFSMNode();

};

#endif