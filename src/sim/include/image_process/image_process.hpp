#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <vector>

#include "image_process/skeleton.hpp"
#include "coordinate_transform/coordinate_transform.hpp"
#include "coordinate_transform/kdl_lib.hpp"

using namespace cv;
using namespace std;
using std::placeholders::_1;


class image_process: public rclcpp::Node
{
  private:
    // 当前仿真模型下两参考点X距离
    const double X_LENGTH = 439.46;

    // 当前仿真模型下两参考点Y距离
    const double Y_LENGTH = 318;

    // 当前仿真模型下两参考点X像素坐标差值
    const int X_PIXEL = 424;
    
    // 当前仿真模型下两参考点Y像素坐标差值
    const int Y_PIXEL = 306;

    //原图图像窗口名
    const string WINDOW_NAME_1  = "Original Image";

    //HSV图像窗口名
    const string WINDOW_NAME_2 = "HSV";

    //衣服区域图像窗口名
    const string WINDOW_NAME_3 = "Garment Area Image";

    //衣服边缘轮廓图像窗口名
    const string WINDOW_NAME_4 = "Garment Contour";

    //inrange函数提取的H范围下限
    const int H_MIN = 35;

    //inrange函数提取的H范围上限
    const int H_MAX = 77;

    //inrange函数提取的S范围下限
    const int S_MIN = 43;  

    //inrange函数提取的S范围上限
    const int S_MAX = 255;

    //inrange函数提取的V范围下限
    const int V_MIN = 46;

    //inrange函数提取的V范围上限
    const int V_MAX =255;

    // 机械臂关节数量
    int joint_num;

    // 坐标转换类实例化
    coordinate_transform coor_trans;

    // 机械臂基坐标系下坐标
    Vector3d robot_point;

    // 机械臂当前各关节角度
    VectorXd cur_joint_states;

    // opencv图像格式
    cv_bridge::CvImagePtr cv_ptr;

    // ROS图像话题订阅变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;

    // 订阅机械臂当前角度话题的变量
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_state_sub;

    //ROS机械臂目标关节角话题发布变量
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_joint_states_pub;

    /*
    * 图像订阅话题回调
    *
    * 用于将ROS topic获取的图像信息转化为opencv能够识别的格式
    * 
    * @param msg 相机话题获取的图像格式
    */
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg);

    /*
    * 图像处理函数,主要是进行仿真下的衣服轮廓提取
    *
    * @param img 输入图像
    */
    void process_image(Mat img);

    void grip_point_extraction(Mat &skeleton_img);

    /*
    * 获取衣服轮廓图像
    *
    * @output 衣服轮廓图像
    */
    static Mat get_skeleton_img();

    /*
    * 获取机械臂当前角度的回调函数
    *
    * @param msg 回调函数信息
    */
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &msg);

    /*
    * 机械臂逆运算，需要将座标点转换为机械臂各关节角
    */
    void inverse_kinematics();

  public:
    /*
    * 构造函数
    *
    * 初始化时订阅相机发布的话题并绑定回调
    * 
    * @param name ROS节点名
    * @param joint_num 机械臂关节数量
    */
    image_process(string name, int joint_num = 7); 

};
#endif