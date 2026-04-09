cmake_minimum_required(VERSION 3.8)
project(task_4)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

install(PROGRAMS
  include/publisher.py
  RENAME publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  include/subscriber.py
  RENAME subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()