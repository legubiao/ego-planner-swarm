//
// Created by tlab-uav on 8/18/24.
//

#include <Eigen/Eigen>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <limits>

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("so3_example");

    auto cmd_pub = node->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);

    rclcpp::Rate rate(100);

    rclcpp::sleep_for(std::chrono::seconds(2));

    while (rclcpp::ok()) {
        /*** example 1: position control ***/
        RCLCPP_INFO(node->get_logger(), "\033[42mPosition Control to (2,0,1) meters\033[0m");
        for (int i = 0; i < 500; i++) {
            quadrotor_msgs::msg::PositionCommand cmd;
            cmd.position.x = 2.0;
            cmd.position.y = 0.0;
            cmd.position.z = 1.0;
            cmd_pub->publish(cmd);

            rate.sleep();
            spin_some(node);
        }

        /*** example 2: velocity control ***/
        RCLCPP_INFO(node->get_logger(), "\033[42mVelocity Control to (-1,0,0) meters/second\033[0m");
        for (int i = 0; i < 500; i++) {
            quadrotor_msgs::msg::PositionCommand cmd;
            cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.velocity.x = -1.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;
            cmd_pub->publish(cmd);

            rate.sleep();
            spin_some(node);
        }

        /*** example 3: acceleration control ***/
        RCLCPP_INFO(node->get_logger(), "\033[42mAcceleration Control to (1,0,0) meters/second^2\033[0m");
        for (int i = 0; i < 500; i++) {
            quadrotor_msgs::msg::PositionCommand cmd;
            cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
            cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
            cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
            cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
            cmd.acceleration.x = 1.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;
            cmd_pub->publish(cmd);

            rate.sleep();
            spin_some(node);
        }
    }

    rclcpp::shutdown();
    return 0;
}
