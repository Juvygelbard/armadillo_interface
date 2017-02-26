#ifndef ROBOT_FSM_H_
#define ROBOT_FSM_H_

class RobotFSMNode{
    public:
        virtual int execute()=0;
};

class RobotFSM: public RobotFSMNode{
    private:
        std::map<int, RobotFSMNode*> *_nodes;
        int _start;

    public:
        RobotFSM();
        int execute();
        bool run();
        void set_start(int id);
        void add_node(int id, RobotFSMNode *node);
        ~RobotFSM();
};

class FuncFSMNode: public RobotFSMNode{     
    public:
        typedef int (*ExecuteFunc)();

        FuncFSMNode(ExecuteFunc func);
        int execute();
        ~FuncFSMNode();
    
    private:
        ExecuteFunc _func;
};

// TODO: implement conjuction-node, disjunction-node, supervised-node (inherite from disj-node)

#endif