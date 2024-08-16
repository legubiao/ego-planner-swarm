//
// Created by biao on 24-8-16.
//

#include <iostream>
#include <cmath>
#include <random>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "quadrotor_msgs/msg/position_command.hpp"

class OdomGenerator final : public rclcpp::Node {
public:
    OdomGenerator() : Node("odom_generator") {
        declare_parameter("init_x", 0.0);
        declare_parameter("init_y", 0.0);
        declare_parameter("init_z", 0.0);

        get_parameter("init_x", _init_x);
        get_parameter("init_y", _init_y);
        get_parameter("init_z", _init_z);

        _cmd_sub = create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "command", 1, std::bind(&OdomGenerator::rcvPosCmdCallBack, this, std::placeholders::_1));
        _odom_pub = create_publisher<nav_msgs::msg::Odometry>("odometry", 1);

        _timer = create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&OdomGenerator::pubOdom, this));
    }

private:
    void rcvPosCmdCallBack(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
        rcv_cmd = true;
        _cmd = *cmd;
    }

    void pubOdom() {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = this->now();
        odom.header.frame_id = "world";

        if (rcv_cmd) {
            odom.pose.pose.position.x = _cmd.position.x;
            odom.pose.pose.position.y = _cmd.position.y;
            odom.pose.pose.position.z = _cmd.position.z;

            Eigen::Vector3d alpha = Eigen::Vector3d(_cmd.acceleration.x, _cmd.acceleration.y, _cmd.acceleration.z) + 9.8
                                    * Eigen::Vector3d(0, 0, 1);
            Eigen::Vector3d xC(cos(_cmd.yaw), sin(_cmd.yaw), 0);
            Eigen::Vector3d yC(-sin(_cmd.yaw), cos(_cmd.yaw), 0);
            Eigen::Vector3d xB = (yC.cross(alpha)).normalized();
            Eigen::Vector3d yB = (alpha.cross(xB)).normalized();
            Eigen::Vector3d zB = xB.cross(yB);
            Eigen::Matrix3d R;
            R.col(0) = xB;
            R.col(1) = yB;
            R.col(2) = zB;
            Eigen::Quaterniond q(R);
            odom.pose.pose.orientation.w = q.w();
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();

            odom.twist.twist.linear.x = _cmd.velocity.x;
            odom.twist.twist.linear.y = _cmd.velocity.y;
            odom.twist.twist.linear.z = _cmd.velocity.z;

            odom.twist.twist.angular.x = _cmd.acceleration.x;
            odom.twist.twist.angular.y = _cmd.acceleration.y;
            odom.twist.twist.angular.z = _cmd.acceleration.z;
        } else {
            odom.pose.pose.position.x = _init_x;
            odom.pose.pose.position.y = _init_y;
            odom.pose.pose.position.z = _init_z;

            odom.pose.pose.orientation.w = 1;
            odom.pose.pose.orientation.x = 0;
            odom.pose.pose.orientation.y = 0;
            odom.pose.pose.orientation.z = 0;

            odom.twist.twist.linear.x = 0.0;
            odom.twist.twist.linear.y = 0.0;
            odom.twist.twist.linear.z = 0.0;

            odom.twist.twist.angular.x = 0.0;
            odom.twist.twist.angular.y = 0.0;
            odom.twist.twist.angular.z = 0.0;
        }

        _odom_pub->publish(odom);
    }

    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr _cmd_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
    rclcpp::TimerBase::SharedPtr _timer;

    quadrotor_msgs::msg::PositionCommand _cmd;
    double _init_x, _init_y, _init_z;
    bool rcv_cmd = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    spin(std::make_shared<OdomGenerator>());
    rclcpp::shutdown();
    return 0;
}
