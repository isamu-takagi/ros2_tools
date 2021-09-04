cmake_minimum_required(VERSION 3.8)
project(nodeif_generators)

find_package(ament_cmake REQUIRED)
ament_export_dependencies(nodeif_cmake)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(CONFIG_EXTRAS "nodeif_generators-extras.cmake")
