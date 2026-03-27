cmake_minimum_required(VERSION 3.8)
project(my_robot_pkg)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeDistance.srv"
)

install(PROGRAMS
  my_robot_pkg/distance_client.py
  RENAME distance_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/distance_server.py
  RENAME distance_serv
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()