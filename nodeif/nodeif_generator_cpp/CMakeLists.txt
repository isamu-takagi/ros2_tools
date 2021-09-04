cmake_minimum_required(VERSION 3.8)
project(nodeif_generator_cpp)

find_package(ament_cmake REQUIRED)
ament_export_dependencies(nodeif_cmake)
ament_index_register_resource("nodeif_generator_packages")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)

install(
  DIRECTORY cmake script template
  DESTINATION share/${PROJECT_NAME}
)

ament_package(CONFIG_EXTRAS "nodeif_generator_cpp-extras.cmake")
