#include "coordinate_transform/kdl_lib.hpp"

kdl_lib::kdl_lib(Chain chain) {
    this -> chain = chain;
}


Frame kdl_lib::kdl_forward_kinematic(VectorXd &joint_states_goal) {
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    Frame result;
    JntArray joint_states(chain.getNrOfJoints());
    joint_states.data = joint_states_goal;

    if (joint_states_goal.rows() == 0) {
        printf("%s \n","Error: joint_states_goal is null");
        return result;
    }
    bool is_success = false;
    is_success = fksolver.JntToCart(joint_states, result);
    if(is_success >= 0){
        std::cout << result <<std::endl;
        printf("%s \n","Succes");
        return result;
    }else{
        printf("%s \n","Error: could not calculate forward kinematics");
        return result;
    }
}

bool kdl_lib::kdl_inverse_kinematic(VectorXd cur_joint_states, Frame position_goal,
                        double (&max)[], double (&min)[], JntArray &result) {
    ChainFkSolverPos_recursive fksolver(chain);
    ChainIkSolverVel_pinv iksolver1v(chain);
    JntArray joint_max(chain.getNrOfJoints());
    JntArray joint_min(chain.getNrOfJoints());
    JntArray joint_states(chain.getNrOfJoints());
    joint_states.data = cur_joint_states;
    for(int ii=0; ii < joint_min.rows(); ii++){
        joint_max(ii) = max[ii];
        joint_min(ii) = min[ii];
    }
    ChainIkSolverPos_NR_JL iksolver(chain, joint_min, joint_max, fksolver, iksolver1v,100,1e-6);
    cout << "pos_goal = " << position_goal << endl;
    int retval = iksolver.CartToJnt(joint_states, position_goal, result);
    if(retval >= 0){
        printf("%s \n","Success");
        return true;
    } else {
        printf("%s \n","Error: could not calculate ik kinematics :(");
        return false;
    }
}