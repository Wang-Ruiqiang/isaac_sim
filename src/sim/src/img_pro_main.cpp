#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include "image_process/image_process.hpp"

using namespace std;

int main(int argc, char **argv)
{
    cout<<"start"<<endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(make_shared<image_process>("image_process"));
    
    rclcpp::shutdown();
    cout<<"end"<<endl;
    return 0;
}