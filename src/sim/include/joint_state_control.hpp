#ifndef JOINT_STATE_CONTROL_H
#define JOINT_STATE_CONTROL_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


using namespace std;
using std::placeholders::_1;

class joint_state_control : public rclcpp::Node
{
  private:
    // 机械臂关节数量
    const int JOINT_NUM = 7;

    // 机械臂关节和夹爪关节数量
    const int JOINT_AND_GRIPPER_NUM = 9;

    bool is_read = true;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_goal_state_sub;

    //ROS话题发布变量
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_control_pub;

    void topic_callback(const std_msgs::msg::Float64MultiArray &msg);

  public:
    joint_state_control();
};

#endif