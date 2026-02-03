cmake_minimum_required(VERSION 3.8)
project(rviz2-panel)

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(DIRECTORY
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

