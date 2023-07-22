cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OpenCV 4.7.0 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(robot_msgs REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(
 include
 ${OpenCV_INCLUDE_DIRS}
)

install(DIRECTORY
  include
  DESTINATION share/${PROJECT_NAME})

add_library(kdl_lib
  src/coordinate_transform/kdl_lib.cpp
)

add_library(img_pro
  src/coordinate_transform/coordinate_transform.cpp
  src/image_process/skeleton.cpp
  src/image_process/image_process.cpp
)

target_include_directories(kdl_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        )

target_include_directories(img_pro PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kdl_lib rclcpp std_msgs sensor_msgs kdl_parser orocos_kdl)

ament_target_dependencies(img_pro rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(img_pro ${OpenCV_LIBS} kdl_lib)

add_executable(kdl_forward_kinematic src/kdl_forward_kinematic.cpp)
target_link_libraries(kdl_forward_kinematic kdl_lib)

add_executable(kdl_inverse_kinematic src/kdl_inverse_kinematic.cpp)
target_link_libraries(kdl_inverse_kinematic kdl_lib)

add_executable(img_pro_main src/img_pro_main.cpp)
ament_target_dependencies(img_pro_main rclcpp std_msgs sensor_msgs cv_bridge image_transport)
target_link_libraries(img_pro_main img_pro)

add_executable(joint_state_control src/joint_state_control.cpp)
ament_target_dependencies(joint_state_control rclcpp std_msgs sensor_msgs)

install(TARGETS
  img_pro_main
  kdl_forward_kinematic
  kdl_inverse_kinematic
  joint_state_control
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  src/ros2_publisher.py
  src/test_multithreading.py
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()
