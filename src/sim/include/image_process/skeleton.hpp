#ifndef SKELENTON_H
#define SKELENTON_H

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.hpp>

#include <vector>
#include <time.h>
#include <math.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm> 

using namespace std;
using namespace cv;

class skeleton {
    public:
        /*
        * 线缆骨骼提取
        *
        * @param input 输出图像
        * @output 处理后的图像
        */
        Mat img_skeleton(Mat input);

    private:
        /*
        * 膨胀操作
        *
        * @param srcImg 输入图像
        * @param size 膨胀范围大小
        * @param times 膨胀次数
        */
        void pre_dilate(Mat & srcImg, int size, int times);

        /*
        * 腐蚀操作
        *
        * @param srcImg 输入图像
        * @param size 腐蚀范围大小
        * @param times 腐蚀次数
        */
        void pre_erode(Mat & srcImg, int size, int times);

        /*
        * 图像细化
        *
        * @param srcImg 输入图像
        */
        void thin_image(Mat & srcImg);

        void thinning_iteration(Mat& img, int iter);

        void thinning(const Mat& src, Mat& dst);
};
#endif