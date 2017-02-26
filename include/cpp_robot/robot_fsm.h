#ifndef ROBOT_FSM_H_
#define ROBOT_FSM_H_

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

// rescives a bector of nodes, returnes the answer from the first node to finish
class DisjFSMNode: public FSMNode{
    private:
        const std::vector<FSMNode> &_nodes;

    public:
        DisjFSMNode(const std::vector<FSMNode> &nodes);
        int execute();
        ~DisjFSMNode();
};

#endif