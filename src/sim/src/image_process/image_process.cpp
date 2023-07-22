#include "image_process/image_process.hpp"

image_process::image_process(string name, int joint_num)
  : Node(name), coor_trans(X_LENGTH, Y_LENGTH, X_PIXEL, Y_PIXEL) {
  this->joint_num = joint_num;
  image_sub = this->create_subscription<sensor_msgs::msg::Image>(
                  "/rgb", 10, bind(&image_process::image_callback, this, _1));
  current_state_sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, bind(&image_process::joint_state_callback, this, _1));
  target_joint_states_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("target_joint_states", 10);
}

void image_process::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);/*使用cvbridge将ros的消息格式sensor_msgs/image复制到cvimage*/
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }
  
  process_image(cv_ptr->image);
}


void image_process::process_image(Mat img) {
  Mat hsv_image;
  cvtColor(img,hsv_image,COLOR_BGR2HSV);
  imshow(WINDOW_NAME_1,img);

  // Rect()函数前两个参数是要截取图片左上角的坐标，如(480,150)是图片左上角的坐标，后两个参数是图片的尺寸
  // 如（800,570）意为图片尺寸是800,570,则需要截取的图片尺寸总大小至少是（1280,720）
  hsv_image = hsv_image(Rect(550,200,600,350));
  imshow(WINDOW_NAME_2, hsv_image);
  Mat garment_area;
  inRange(hsv_image,Scalar(H_MIN,S_MIN,V_MIN),Scalar(H_MAX,S_MAX,V_MAX),garment_area);
  imshow(WINDOW_NAME_3, garment_area);

  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  findContours(garment_area, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
  Mat skeleton_img = Mat::zeros(garment_area.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar(255, 255, 255);
        drawContours(skeleton_img, contours, (int)i, color, 1, LINE_8, hierarchy, 0);
    }
    imshow(WINDOW_NAME_4, skeleton_img);
    cvtColor(skeleton_img, skeleton_img, CV_BGR2GRAY);
    grip_point_extraction(skeleton_img);
    waitKey(10);
}

void image_process::grip_point_extraction(Mat &skeleton_img) {
  static vector<Point2i> skeleton_point_set;
  findNonZero(skeleton_img, skeleton_point_set);
  vector<int> x_min_pixel_point(3);
  coordinate_transform::find_minimum_point(true, skeleton_point_set, skeleton_point_set.size(), x_min_pixel_point);
  cout << "x_min_point = " << x_min_pixel_point[0] << endl;
  cout << "y_min_point = " << x_min_pixel_point[1] << endl;
  Vector3d camera_point =  coor_trans.pixel_coordinate_to_camera_coordinate(x_min_pixel_point);
  robot_point = coor_trans.camera_coordinate_to_robot_coordinate(camera_point);
  // cout << "camera_point[0] = " << camera_point(0, 0) << endl;
  // cout << "camera_point[1] = " << camera_point(1, 0) << endl;
  // cout << "camera_point[2] = " << camera_point(2, 0) << endl;
  // cout << "robot_point[0] = " << robot_point(0, 0) << endl;
  // cout << "robot_point[1] = " << robot_point(1, 0) << endl;
  // cout << "robot_point[2] = " << robot_point(2, 0) << endl;
}

void image_process::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr &msg) {
  cur_joint_states.resize(joint_num);
  for (int i = 0; i < cur_joint_states.size(); i++) {
    cur_joint_states(i, 0) = msg->position[i];
  }
  inverse_kinematics();
  // cout << "positon[0] = " << cur_joint_states(0, 0) <<endl;
  // cout << "positon[1] = " << cur_joint_states(1, 0) <<endl;
  // cout << "positon[2] = " << cur_joint_states(2, 0) <<endl;
  // cout << "positon[3] = " << cur_joint_states(3, 0) <<endl;
  // cout << "positon[4] = " << cur_joint_states(4, 0) <<endl;
  // cout << "positon[5] = " << cur_joint_states(5, 0) <<endl;
  // cout << "positon[6] = " << cur_joint_states(6, 0) <<endl;
}

void image_process::inverse_kinematics() {
  KDL:: Tree panda_tree;
  kdl_parser::treeFromFile("/home/ruiqiang/WorkSpace/Ruiqiang/humble_ws/src/sim/src/panda_description/panda.urdf", panda_tree);
  Chain chain;
  bool exit_value = panda_tree.getChain("panda_link0","panda_link8",chain);
  if (!exit_value) {
    cout << "panda_tree.getChain error" <<endl;
    return;
  }

  kdl_lib kdl(chain);
  Frame target_pos;
  for (int i =0; i < 3; i++) {
    target_pos.p[i] = robot_point[i];
  }

  // 调整机械臂末端位姿
  Rotation rotX(1, 0, 0, 0, -1, 0, 0, 0, -1);
  Rotation rotZ(cos(3.14*0.53/1.8), -sin(3.14*0.53/1.8), 0, sin(3.14*0.53/1.8), cos(3.14*0.53/1.8), 0, 0, 0, 1);
  target_pos.M = rotX * rotZ;
// self.default_joints = [0.0, 0.718, -0.0, -1.9, -0.0, 2.6, 0.7, 0.2, 0.2]

  // 每个关节允许的最大角度与最小角度
  // double minjp[7] = {-2.9671,-1.8326,-2.9671,-3.1416,-2.9671,-0.0873,-2.9671};
  // double maxjp[7] = { 2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 3.8223, 2.9671};
  double minjp[joint_num] = {-2.9671,-1.8326,-2.9671,-3.1416,-2.9671,-0.0873, -2.9671};
  double maxjp[joint_num] = { 2.9671, 1.8326, 2.9671, 0.0873, 2.9671, 3.8223, 2.9671};
  JntArray joint_sum_result(chain.getNrOfJoints());

  bool is_success = kdl.kdl_inverse_kinematic(cur_joint_states, target_pos, maxjp, minjp, joint_sum_result);
  if (!is_success) {
    cout << "kdl_inverse_kinematic error" <<endl;
    return;
  }

  vector<double> joint_goal(joint_num);
  for (int i = 0; i < joint_sum_result.rows(); i++) {
    joint_goal[i] = joint_sum_result(i);
    cout << joint_sum_result(i) << endl;
  }

  std_msgs::msg::Float64MultiArray message;
  message.data = joint_goal;
  target_joint_states_pub->publish(message);
}