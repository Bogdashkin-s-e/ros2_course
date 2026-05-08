cmake_minimum_required(VERSION 3.8)
project(hm7_slam_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(PROGRAMS
  hm7_slam_pkg/slam_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME slam_node
)

install(PROGRAMS
  hm7_slam_pkg/wall_follower.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wall_follower
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
