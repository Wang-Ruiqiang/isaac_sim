#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "joint_state_control.hpp"


using namespace std;
using std::placeholders::_1;

joint_state_control::joint_state_control()
    : Node("joint_state_control")
{
    joint_control_pub = this->create_publisher<sensor_msgs::msg::JointState>("joint_command", 10);
    joint_goal_state_sub = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/target_joint_states", 10, bind(&joint_state_control::topic_callback, this, _1));
}

rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_goal_state_sub;

void joint_state_control::topic_callback(const std_msgs::msg::Float64MultiArray &msg) 
{  
    if(is_read) {
        vector<double> joint_target_data = msg.data;
    
        joint_target_data.push_back(0.01);
        joint_target_data.push_back(0.01);
        vector<string> joint_name = {
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
            "panda_finger_joint1",
            "panda_finger_joint2"
        };
        for (int i = 0; i < 9; i++) {
            cout << joint_target_data[i] << endl;
        }

        vector<double> velocity = {0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05};
        sensor_msgs::msg::JointState position_message;
        position_message.name = joint_name;
        position_message.position = joint_target_data;
        joint_control_pub->publish(position_message);
        
        is_read = false;
    }
    
}
    



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<joint_state_control>());
  
    rclcpp::shutdown();
  return 0;
}