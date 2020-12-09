cmake_minimum_required(VERSION 3.5)
project(global_planner)

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(dependencies
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        OpenCV
        cv_bridge
        image_geometry
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
include_directories(include)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
        src/GlobalPlanner.cpp)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(global_planner_main src/main.cpp)
target_link_libraries(global_planner_main ${library_name})
ament_target_dependencies(global_planner_main ${dependencies})

install(TARGETS
        global_planner_main ${library_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/
        )


ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
