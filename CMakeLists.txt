cmake_minimum_required(VERSION 3.5)
project(tkgism_ros_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(
 ${rclcpp_INCLUDE_DIRS}
 ${Qt5Core_INCLUDE_DIRS}
 ${Qt5Quick_INCLUDE_DIRS}
)

add_executable(interface_relations
  src/interface_relations/main.cpp
  src/interface_relations/graph.hpp
  src/interface_relations/graph.cpp
)
ament_target_dependencies(interface_relations rclcpp)

add_executable(interface_relations_gui
  src/interface_relations/maingui.cpp
  src/interface_relations/graph.hpp
  src/interface_relations/graph.cpp
  src/interface_relations/window.hpp
  src/interface_relations/window.cpp
)
ament_target_dependencies(interface_relations_gui rclcpp)
target_link_libraries(interface_relations_gui Qt5::Core Qt5::Widgets)

install(
  TARGETS interface_relations interface_relations_gui
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
