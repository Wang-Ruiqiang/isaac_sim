// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <coordinate_transform/kdl_lib.hpp>
#include <stdio.h>
#include <iostream>
 
using namespace KDL;
 
 
int main()
{
    //Definition of a kinematic chain & add segments to the chain
    // KDL::Chain chain;
    // chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Vector(0.0,0.0,1.020))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.480))));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.645))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));
    // chain.addSegment(Segment(Joint(Joint::RotX),Frame(Vector(0.0,0.0,0.120))));
    // chain.addSegment(Segment(Joint(Joint::RotZ)));

    KDL:: Tree panda_tree;
    kdl_parser::treeFromFile("/home/ruiqiang/WorkSpace/humble_ws/src/simulation/src/panda_description/panda.urdf", panda_tree);
    bool exit_value;
    Chain chain;
    exit_value = panda_tree.getChain("panda_link0","panda_link8",chain);

 
    // // Create solver based on kinematic chain
    // ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
 
    // // Create joint array
    // unsigned int nj = chain.getNrOfJoints();
    KDL::JntArray jointpositions = JntArray(chain.getNrOfJoints());
    VectorXd joint_states(7,1);
    // Assign some values to the joint positions
    for(unsigned int i=0;i<chain.getNrOfJoints();i++){
        float myinput;
        printf ("Enter the position of joint %i: ",i);
        scanf ("%e",&myinput);
        joint_states(i)=(double)myinput;
    }

    kdl_lib kdl(chain);
    Frame result;
    result = kdl.kdl_forward_kinematic(joint_states);
    std::cout << "jointpositions = " << result <<std::endl;

 
    // // Create the frame that will contain the results
    // KDL::Frame cartpos;    
 
    // // Calculate forward position kinematics
    // bool kinematics_status;
    // kinematics_status = fksolver.JntToCart(jointpositions,cartpos);
    // if(kinematics_status>=0){
    //     std::cout << cartpos <<std::endl;
    //     printf("%s \n","Succes, thanks KDL!");
    // }else{
    //     printf("%s \n","Error: could not calculate forward kinematics :(");
    // }
}