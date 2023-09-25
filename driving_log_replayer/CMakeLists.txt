cmake_minimum_required(VERSION 3.14)
project(driving_log_replayer)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(obstacle_segmentation_evaluator_component SHARED
  src/obstacle_segmentation_evaluator_component.cpp
)
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
  find_package(ament_cmake_pytest)
  ament_add_pytest_test(driving_log_replayer
    test
  )
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  scripts/perception_evaluator_node.py
  scripts/perception_database_result.py
  scripts/perception_2d_evaluator_node.py
  scripts/traffic_light_evaluator_node.py
  scripts/obstacle_segmentation_evaluator_node.py
  scripts/performance_diag_evaluator_node.py
  scripts/localization_evaluator_node.py
  scripts/yabloc_evaluator_node.py
  scripts/eagleye_evaluator_node.py
  scripts/ar_tag_based_localizer_evaluator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
