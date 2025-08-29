cmake_minimum_required(VERSION 3.8)
project(mechanical_press)

find_package(ament_cmake REQUIRED)

# Install subdirectory packages
install(DIRECTORY
  ../mechanical_press_core
  ../mechanical_press_bringup
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()