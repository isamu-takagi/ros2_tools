cmake_minimum_required(VERSION 3.8)
project(nodeif_user1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

message(STATUS "[NODEIF] ${PROJECT_NAME} CMakeList")

nodeif_generate(${PROJECT_NAME}
  nodeif/talker/SendChatter
  nodeif/listener/RecvChatter
)

#ament_auto_add_library(${PROJECT_NAME} SHARED
#  src/talker.cpp
#  src/listener.cpp
#)
#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "Talker"
#  EXECUTABLE talker
#)
#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "Listener"
#  EXECUTABLE listener
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
