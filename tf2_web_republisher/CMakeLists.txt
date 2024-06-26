cmake_minimum_required(VERSION 3.22)
project(tf2_web_republisher)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
    builtin_interfaces
    eigen3_cmake_module
    rclcpp
    rclcpp_action
    tf2
    tf2_eigen
    tf2_geometry_msgs
    tf2_msgs
    tf2_ros
    tf2_web_republisher_msgs)

foreach(package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${package} REQUIRED)
endforeach()

add_library(tf2_web_republisher src/tf2_web_republisher.cpp)
target_include_directories(
  tf2_web_republisher
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(tf2_web_republisher_node src/tf2_web_republisher_node.cpp)
target_link_libraries(tf2_web_republisher_node tf2_web_republisher)

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(
  TARGETS tf2_web_republisher_node
  EXPORT export_test_tf2_web_republisher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_package()
