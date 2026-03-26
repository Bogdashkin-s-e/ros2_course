cmake_minimum_required(VERSION 3.8)
project(my_robot_pkg)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
)

install(PROGRAMS
  my_robot_pkg/sensor_publisher.py
  RENAME sensor_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/sensor_subscriber.py
  RENAME sensor_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()