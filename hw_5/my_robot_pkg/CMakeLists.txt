cmake_minimum_required(VERSION 3.8)
project(task_5_turtle)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/KillTurtle.srv"
  "srv/SpawnTurtle.srv"
)

install(PROGRAMS
  include/client.py
  RENAME client
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  include/server.py
  RENAME server
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()