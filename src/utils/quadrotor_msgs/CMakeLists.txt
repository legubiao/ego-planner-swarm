cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        "msg/AuxCommand.msg"
        "msg/Corrections.msg"
        "msg/Gains.msg"
        "msg/LQRTrajectory.msg"
        "msg/Odometry.msg"
        "msg/OutputData.msg"
        "msg/PolynomialTrajectory.msg"
        "msg/PositionCommand.msg"
        "msg/PPROutputData.msg"
        "msg/Serial.msg"
        "msg/SO3Command.msg"
        "msg/StatusData.msg"
        "msg/TRPYCommand.msg"
        DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

ament_package()
