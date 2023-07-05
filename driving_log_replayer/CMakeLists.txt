cmake_minimum_required(VERSION 3.14)
project(driving_log_replayer)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(obstacle_segmentation_evaluator_component SHARED
  src/obstacle_segmentation_evaluator_component.cpp
)
# workaround to allow deprecated header to build on both galactic and humble
if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  target_compile_definitions(obstacle_segmentation_evaluator_component PUBLIC
    USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
  )
endif()
# Workaround for lanelet2-core@1.2.1
if(TARGET lanelet2_core::lanelet2_core)
  get_target_property(lanelet2_core_INCLUDE_DIRECTORIES lanelet2_core::lanelet2_core INTERFACE_INCLUDE_DIRECTORIES)
  include_directories(SYSTEM
    ${lanelet2_core_INCLUDE_DIRECTORIES}
  )
endif()
rclcpp_components_register_node(obstacle_segmentation_evaluator_component
  PLUGIN "driving_log_replayer::ObstacleSegmentationEvaluatorComponent"
  EXECUTABLE obstacle_segmentation_evaluator_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/perception_evaluator_node.py
  scripts/perception_database_result.py
  scripts/perception_2d_evaluator_node.py
  scripts/traffic_light_evaluator_node.py
  scripts/obstacle_segmentation_evaluator_node.py
  scripts/performance_diag_evaluator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
