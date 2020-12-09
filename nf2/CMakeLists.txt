cmake_minimum_required(VERSION 3.5)
project(nf2)
# if it loads a wrong python from anaconda, diasble anaconda, delete the package ,restart, create the package again
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
find_package(OpenCV REQUIRED)

set(dependencies
        rclcpp
        OpenCV
        )

include_directories(include)
#set(library_name ${PROJECT_NAME})

add_executable(run_nf2 src/run_nf2.cpp)
target_include_directories(run_nf2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
#target_link_libraries(run_nf2 ${library_name})
ament_target_dependencies(run_nf2 ${dependencies})

install(TARGETS run_nf2)
#install(DIRECTORY include/
#        DESTINATION include/
#        )
#
#ament_export_include_directories(include)
#ament_export_libraries(${library_name})
#ament_export_dependencies(${dependencies})

ament_package()
