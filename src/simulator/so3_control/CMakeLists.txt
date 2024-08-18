cmake_minimum_required(VERSION 3.8)
project(so3_control)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Add library for SO3Control
add_library(${PROJECT_NAME} src/SO3Control.cpp)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Add executable for example
add_executable(so3_example src/so3_example.cpp)
target_link_libraries(so3_example ${PROJECT_NAME})
ament_target_dependencies(so3_example rclcpp quadrotor_msgs)

# Add executable for SO3ControlNode
add_executable(so3_control_node src/SO3ControlNode.cpp)
target_link_libraries(so3_control_node ${PROJECT_NAME})
ament_target_dependencies(so3_control_node rclcpp rclcpp_components quadrotor_msgs tf2 sensor_msgs tf2_geometry_msgs)


install(TARGETS
        so3_example
        so3_control_node
        DESTINATION lib/${PROJECT_NAME})


ament_package()
