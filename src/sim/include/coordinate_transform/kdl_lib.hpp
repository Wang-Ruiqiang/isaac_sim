#ifndef KDL_LIB_H
#define KDL_LIB_H

#include <cstddef>
#include <iostream>
#include <stdio.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>
#include <urdf_model/model.h>

using namespace std;
using namespace KDL;
using namespace Eigen;

/*
* KDL求正逆解类
*
*/
class kdl_lib {
    private:
        Chain chain;

    public:
        /*
        * 构造函数
        *
        * @param chain 机械臂结构链，可使用tree从URDF文件直接导入
        */
        kdl_lib(Chain chain);

        /*
        * kdl正向解方法
        *
        * @output 求解的坐标，输出为3*3的旋转矩阵以及3*1的空间xyz坐标
        */
        Frame kdl_forward_kinematic(VectorXd &joint_states);

        /*
        * kdl逆向解方法（通过空间坐标求解关节角）
        *
        * @param 当前机械臂关节角度
        * @param position_goal 目标位置
        * @param max 每个关节运动角度上限
        * @param min 每个关节运动角度下限
        * @param result 求解的关节角
        * @output 求解是否成功
        */
        bool kdl_inverse_kinematic(VectorXd cur_joint_states, Frame position_goal,
                        double (&max)[], double (&min)[], JntArray &result);
};
#endif