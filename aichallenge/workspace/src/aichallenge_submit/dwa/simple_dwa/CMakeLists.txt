cmake_minimum_required(VERSION 3.8)
project(simple_dwa)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(simple_dwa
  src/main.cpp
  src/controller.cpp
  src/course.cpp
  src/dwa.cpp
  src/dwa_node.cpp
  src/obstacle.cpp
  src/path.cpp
  src/robot.cpp
  src/simulator.cpp
  src/utils.cpp
)

install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()