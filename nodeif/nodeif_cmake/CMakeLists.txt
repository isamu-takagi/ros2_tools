cmake_minimum_required(VERSION 3.8)
project(nodeif_cmake)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "nodeif_cmake-extras.cmake")
