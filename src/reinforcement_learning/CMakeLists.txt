cmake_minimum_required(VERSION 3.8)
project(reinforcement_learning)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(DIRECTORY
  src
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  src/test.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
