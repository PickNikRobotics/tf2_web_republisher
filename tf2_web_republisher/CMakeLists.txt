cmake_minimum_required(VERSION 3.22)
project(tf2_web_republisher)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
        Boost
        Eigen3
        rclcpp
        tf2_ros
        tf2
        tf2_msgs
        builtin_interfaces
        tf2_eigen
        rclcpp_action
        tf2_geometry_msgs
        tf2_web_republisher_msgs
)

foreach (package IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
    find_package(${package} REQUIRED)
endforeach ()

add_executable(${PROJECT_NAME} src/tf_web_republisher.cpp)
target_include_directories(
        ${PROJECT_NAME}
        PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})


install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
