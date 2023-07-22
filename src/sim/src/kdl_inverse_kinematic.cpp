// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
#include <stdio.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
 
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/frames_io.hpp>
// #include <kdl_parser/kdl_parser.hpp>

#include "coordinate_transform/kdl_lib.hpp"

using namespace KDL;
using namespace std;
using namespace Eigen;
 
 
int main( int argc, char** argv )
{
    KDL:: Tree panda_tree;
    kdl_parser::treeFromFile("/home/ruiqiang/WorkSpace/Ruiqiang/humble_ws/src/sim/src/panda_description/panda.urdf", panda_tree);
    bool exit_value;
    Chain chain;
    exit_value = panda_tree.getChain("panda_link0","panda_link8",chain);

    // KDL::Chain chain;
    // chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));

    ChainFkSolverPos_recursive fksolver(chain);
    // ChainIkSolverVel_pinv iksolver1v(chain);//Inverse velocity solver

    JntArray joint_max(chain.getNrOfJoints());
    JntArray joint_min(chain.getNrOfJoints());
    double minjp[7] = {-2.9671,-1.8326,-2.9671,-3.1416,-2.9671,-0.0873,-2.9671};
    double maxjp[7] = { 2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 3.8223, 2.9671};
    // for(int ii=0; ii < joint_min.rows(); ii++){
    //     joint_max(ii) = maxjp[ii];
    //     joint_min(ii) = minjp[ii];
    // }

    // ChainIkSolverPos_NR_JL iksolver(chain, joint_min, joint_max, fksolver, iksolver1v,100,1e-6);

    JntArray joint_sum(chain.getNrOfJoints());
    std::cout << "getNrOfJoints = " << chain.getNrOfJoints() << std::endl;
    Frame pos_goal;
    // joint_sum.data.setRandom();
    // joint_sum.data(0) = 0.012;
    // joint_sum.data(1) = -0.6;
    // joint_sum.data(2) = 0;
    // joint_sum.data(3) = -2.8105;
    // joint_sum.data(4) = 0;
    // joint_sum.data(5) = 3.0312;
    // joint_sum.data(6) = 0.7853;

    // joint_sum.data(0) = 0.0;
    // joint_sum.data(1) = -1.16;
    // joint_sum.data(2) = -0.0;
    // joint_sum.data(3) = -2.3;
    // joint_sum.data(4) = -0.0;
    // joint_sum.data(5) = 1.6;
    // joint_sum.data(6) = 1.1;

    // joint_sum.data(0) = 0;
    // joint_sum.data(1) = 0;
    // joint_sum.data(2) = 0;
    // joint_sum.data(3) = 0;
    // joint_sum.data(4) = 0;
    // joint_sum.data(5) = 0;
    // joint_sum.data(6) = 0;

    joint_sum.data(0) = 0;
    joint_sum.data(1) = 0.71;
    joint_sum.data(2) = 0;
    joint_sum.data(3) = -1.9;
    joint_sum.data(4) = 0;
    joint_sum.data(5) = 2.6;
    joint_sum.data(6) = 0.7;

    // joint_sum.data *= M_PI;
    fksolver.JntToCart(joint_sum, pos_goal);
    std::cout << "pos_goal = " << pos_goal << std::endl;
    
    JntArray joint_sum_init(chain.getNrOfJoints());
    JntArray joint_sum_result(chain.getNrOfJoints());
    joint_sum_init.data(0) = 0.012;
    joint_sum_init.data(1) = -0.5697;
    joint_sum_init.data(2) = 0;
    joint_sum_init.data(3) = -2.8105;
    joint_sum_init.data(4) = 0;
    joint_sum_init.data(5) = 3.0312;
    joint_sum_init.data(6) = 0.7853;

    // joint_sum_init.data.setRandom();
    VectorXd joint_states(7, 1);
    for(unsigned int i = 0; i < 7;i++){
        joint_states(i, 0)=joint_sum_init.data(i);
    }
    
    

    kdl_lib kdl(chain);
    bool is_success = kdl.kdl_inverse_kinematic(joint_states, pos_goal, maxjp, minjp, joint_sum_result);
    if (is_success) {
        for (int i = 0; i < joint_sum_result.rows(); i++) {
            std::cout << joint_sum_result(i) << std::endl;
        }
    } else {
        printf("%s \n","Error: could not calculate ik kinematics :(");
    }

    // joint_sum_init.data(0) = 0.0;
    // joint_sum_init.data(1) = -1.16;
    // joint_sum_init.data(2) = -0.0;
    // joint_sum_init.data(3) = -2.3;
    // joint_sum_init.data(4) = -0.0;
    // joint_sum_init.data(5) = 1.6;
    // joint_sum_init.data(6) = 1.1;
    // joint_sum_init.data *= M_PI;

    
    // int retval = iksolver.CartToJnt(joint_sum_init, pos_goal, joint_sum_result);
    // if(retval>=0){
    //     for (int i = 0; i < joint_sum_result.rows(); i++) {
    //         std::cout << joint_sum_result(i) << std::endl;
    //     }
    //     printf("%s \n","Succes, thanks KDL!");
    // }else{
    //     printf("%s \n","Error: could not calculate ik kinematics :(");
    // }

}
