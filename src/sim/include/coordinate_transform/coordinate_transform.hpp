#ifndef COORDINATE_TRANSFORMATION_H
#define COORDINATE_TRANSFORMATION_H

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/opencv.hpp>

#include <vector>
#include <time.h>
#include <math.h>
#include <algorithm>

using namespace std;
using namespace cv;
using namespace Eigen;

/*
* 坐标变换类
*
*/
class coordinate_transform
{
    private:
        // 相机坐标系X轴与机械臂坐标系X轴的距离
        const double  DIS_WITH_CAMERA_AND_ROBOT_X = 0.62;

        //相机坐标系Z轴与机械臂坐标系Z轴的距离
        const double  DIS_WITH_CAMERA_AND_ROBOT_Z = 1.53;

        // 选取的两个参考点的x距离
        double x_length;

        // 选取的两个参考点的y距离
        double y_length;

        // 选取的两个参考点的x轴像素差
        int x_pixel;

        // 选取的两个参考点的y轴像素差
        int y_pixel;

        // 像素坐标系x轴1mm有多少像素点
        double x_pixels_permm;

        // 像素坐标系y轴1mm有多少像素点
        double y_pixels_permm;
    public:
        /*
        * 构造函数，参数用于确定每个像素点代表的实际长度
        *
        * @param x_length 选中的x轴参考长度
        * @param y_length 选中的y轴参考长度
        * @param x_pixel 在x_length对应的长度下的像素点数量
        * @param y_pixel 在y_length对应的长度下像素点的数量
        */
        coordinate_transform(double x_length, double y_length, int x_pixel, int y_pixel);

        /*
        * 像素坐标系下的坐标转换到相机坐标系的坐标，注意仿真环境里相机xyz轴的方向
        *
        * @param pixel_point 像素坐标系下的坐标
        * @output 相机坐标系下坐标
        */
        Vector3d pixel_coordinate_to_camera_coordinate(vector<int> &pixel_point);

        /*
        *相机坐标转换到机器人坐标系下坐标
        *
        *@param cameaPoint 目标点在相机坐标系下坐标，为3*1矩阵
        *@output 目标点在机器人其坐标系下坐标，为3*1矩阵
        */
        Vector3d camera_coordinate_to_robot_coordinate(Vector3d &cameraPoint);

        /*
        * 查找轮廓x或者y坐标的最小点
        *
        * @param is_x_axis 是否查找x轴最小点，false为查找Y轴最小座标点
        * @param point_set 轮廓点集
        * @param length 轮廓点集长度
        * @output_array 输出结果
        */
        static void find_minimum_point(bool is_x_axis, vector<Point2i>  &point_set, int length, vector<int> &output_array);

        /*
        * 查找轮廓x或者y坐标的最大点
        *
        * @param is_x_axis 是否查找x轴最大点，false为查找Y轴最大座标点
        * @param point_set 轮廓点集
        * @param length 轮廓点集长度
        * @output_array 输出结果
        */
        static void find_maximum_point(bool is_x_axis, vector<Point2i>  &point_set, int length, vector<int> &output_array);
};

#endif