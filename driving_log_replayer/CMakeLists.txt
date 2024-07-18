cmake_minimum_required(VERSION 3.14)
project(driving_log_replayer)

find_package(ament_cmake_python REQUIRED)
find_package(autoware_cmake REQUIRED)
autoware_package()

set(ROS_REQUIREMENTS "${CMAKE_CURRENT_SOURCE_DIR}/../requirements.txt")
execute_process(COMMAND bash -c "python3 -m pip install -r ${ROS_REQUIREMENTS}")

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
  scripts/ground_segmentation_evaluator_node.py
  scripts/obstacle_segmentation_evaluator_node.py
  scripts/performance_diag_evaluator_node.py
  scripts/localization_evaluator_node.py
  scripts/yabloc_evaluator_node.py
  scripts/eagleye_evaluator_node.py
  scripts/ar_tag_based_localizer_evaluator_node.py
  scripts/annotationless_perception_evaluator_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  sample
)