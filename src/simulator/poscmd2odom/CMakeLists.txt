cmake_minimum_required(VERSION 3.8)
project(poscmd2odom)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_executable(odom_generator src/odom_generator.cpp)
target_include_directories(odom_generator
        PUBLIC
        ${EIGEN3_INCLUDE_DIR})
ament_target_dependencies(odom_generator
        rclcpp
        nav_msgs
        quadrotor_msgs)

install(TARGETS
        odom_generator
        DESTINATION lib/${PROJECT_NAME})

ament_package()
