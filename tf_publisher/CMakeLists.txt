cmake_minimum_required(VERSION 3.5)
project(tf_publisher)

set(FastRTPS_INCLUDE_DIR /opt/ros/foxy/include)
set(FastRTPS_LIBRARY_RELEASE /opt/ros/foxy/lib/libfastrtps.so)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
        rclcpp
        tf2_ros
        tf2
        geometry_msgs)

include_directories(include)
set(library_name ${PROJECT_NAME})

add_library(${library_name} SHARED
        src/tf_publisher.cpp)
ament_target_dependencies(${library_name} ${dependencies})

add_executable(tf_publisher_main src/tf_publisher_main.cpp)
target_link_libraries(tf_publisher_main ${library_name})
ament_target_dependencies(tf_publisher_main ${dependencies})

install(TARGETS
        tf_publisher_main ${library_name}
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
