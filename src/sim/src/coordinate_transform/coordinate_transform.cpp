#include  "coordinate_transform/coordinate_transform.hpp"

coordinate_transform::coordinate_transform(double x_length, double y_length, int x_pixel, int y_pixel)
{
   // garment 左右袖口距离 0.439456m 左右像素(32,42) (456, 42)
   // garment 长度 0.318m 中心点到最下方长度0.306067m 上下像素(273或215,15) (129,321)
   // 相机像素1280*720
   this->x_length = x_length;
   this->y_length = y_length;
   this->x_pixel = x_pixel;
   this->y_pixel = y_pixel;
   x_pixels_permm = this->x_pixel/this->x_length;    //定义为1mm有多少像素点
   y_pixels_permm = this->y_pixel/this->y_length;
}

Vector3d coordinate_transform::pixel_coordinate_to_camera_coordinate(vector<int> &pixel_point)
{
   vector<double> camera_point(3);

   camera_point[0] = (double)(pixel_point[0] + 550 -640);                                       
   camera_point[1] = -(double)(pixel_point[1] + 200 -360);
   camera_point[2] = -1.40; //机械臂移动点需要与地面有一定高度差
   VectorXd result(3,1);
   camera_point[0] = camera_point[0]/x_pixels_permm;
   camera_point[1] = camera_point[1]/y_pixels_permm;
   result << camera_point[0]/1000,
        camera_point[1]/1000,
        camera_point[2];
   
   return result;
}

Vector3d coordinate_transform::camera_coordinate_to_robot_coordinate(Vector3d &camera_point)
{
   /* MatrixXd X(3,3);      旋转矩阵公式
    * MatrixXd Y(3,3);
    * MatrixXd Z(3,3);
    * X<<1,0,0,
    *    0,cos(Euler[2]),-sin(Euler[2]),    绕X轴旋转角度
    *   0,sin(Euler[2]),cos(Euler[2]);
    * Y<<cos(Euler[0]),0,sin(Euler[0]),
    *    0,1,0,
    *    -sin(Euler[0]),0,cos(Euler[0]);
    * Z<<cos(Euler[1]),-sin(Euler[1]),0,    绕Y轴旋转角度
    *    sin(Euler[1]),cos(Euler[1]),0,     绕Z轴旋转角度
    *    0,0,1;
    * R=Z*Y*X;
    *
    * R<<(1-2*y*y-2*z*z),(2*x*y-2*w*z),(2*x*z+2*w*y),     四元数转化为旋转矩阵
    *    (2*x*y+2*w*z),(1-2*x*x-2*z*z),(2*y*z-2*w*x),
    *    (2*x*z-2*w*y),(2*y*z+2*w*x),(1-2*x*x-2*y*y);
    * 
    *Example : 
    *   先计算相机坐标与机器人坐标平移距离 
    *       input(0,0)=input(0,0)+105-10;
    *       input(1,0)=input(1,0)+158.604;
    *       input(2,0)=input(2,0)-599-105;
    *   在计算旋转关系 
    *      X<<1,0,0,
    *         0,cos(-3.1415926*5/36),-sin(-3.1415926*5/36),
    *         0,sin(-3.1415926*5/36),cos(-3.1415926*5/36);
    *      Y<<cos(-1.5707963),0,sin(-1.5707963),
    *         0,1,0,
    *         -sin(-1.5707963),0,cos(-1.5707963);
    *
    *      Z<<cos(3.1415926*5/36),-sin(3.1415926*5/36),0,
    *         sin(3.1415926*5/36),cos(3.1415926*5/36),0,
    *         0,0,1;
    *
    *        R=X*Y*Z;
    *
    *        result=R*input;
    *        return result;
   */

   //X,Y,Z轴旋转矩阵以及综合旋转矩阵
   MatrixXd rotation_MatrixXZ(3,3);
   MatrixXd rotation_MatrixXY(3,3);
   MatrixXd rotation_MatrixXX(3,3);
   rotation_MatrixXZ << 0,-1,0,
                        1,0,0,
                        0,0,1;

   VectorXd result_coordinate(3,1);     
   result_coordinate << camera_point(0,0),
                       camera_point(1,0) - DIS_WITH_CAMERA_AND_ROBOT_X,
                       camera_point(2,0) + DIS_WITH_CAMERA_AND_ROBOT_Z;
   result_coordinate = rotation_MatrixXZ * result_coordinate;             
   return result_coordinate;
}

void coordinate_transform::find_minimum_point(bool is_x_axis, vector<Point2i>  &point_set, int length, vector<int> &output_array){
   if (length == 0) {
      return;
   } 
   if (length == 1) {
      output_array[0] = point_set[0].x;
      output_array[1] = point_set[0].y;
      return;
   }
   output_array[0] = point_set[0].x;
   output_array[1] = point_set[0].y;
   if(is_x_axis) {
      for (int i = 0; i < length; i++) {
         if (point_set[i].x < output_array[0]) {
            output_array[0] = point_set[i].x;
            output_array[1] = point_set[i].y;
         }
      }
      return;
   } else {
      for (int i = 1; i < length; i++) {
         if (point_set[i].y < output_array[1]) {
            output_array[0] = point_set[i].x;
            output_array[1] = point_set[i].y;
         }
      }
      return;
   }
}

void coordinate_transform::find_maximum_point(bool is_x_axis, vector<Point2i>  &point_set, int length, vector<int> &output_array){
   if (length == 0) {
      return;
   } 
   if (length == 1) {
      output_array[0] = point_set[0].x;
      output_array[1] = point_set[0].y;
      return;
   }
   output_array[0] = point_set[0].x;
   output_array[1] = point_set[0].y;
   if(is_x_axis) {
      for (int i = 0; i < length; i++) {
         if (point_set[i].x > output_array[0]) {
            output_array[0] = point_set[i].x;
            output_array[1] = point_set[i].y;
         }
      }
      return;
   } else {
      for (int i = 1; i < length; i++) {
         if (point_set[i].y > output_array[1]) {
            output_array[0] = point_set[i].x;
            output_array[1] = point_set[i].y;
         }
      }
      return;
   }
}