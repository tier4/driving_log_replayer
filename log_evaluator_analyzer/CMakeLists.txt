cmake_minimum_required(VERSION 3.14)
project(log_evaluator_analyzer)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_python_install_package(${PROJECT_NAME})

ament_auto_package(INSTALL_TO_SHARE
  ${PROJECT_NAME}/config
)
