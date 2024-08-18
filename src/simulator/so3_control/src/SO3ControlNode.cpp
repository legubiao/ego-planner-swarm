#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <quadrotor_msgs/msg/corrections.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/so3_command.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <so3_control/SO3Control.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class SO3ControlNode {
public:
    SO3ControlNode(rclcpp::Node::SharedPtr node)
        : node_(node),
          position_cmd_updated_(false),
          position_cmd_init_(false),
          des_yaw_(0),
          des_yaw_dot_(0),
          current_yaw_(0),
          enable_motors_(true),
          use_external_yaw_(false) {
        node_->declare_parameter<std::string>("quadrotor_name", "quadrotor");
        node_->declare_parameter<double>("mass", 0.5);
        node_->declare_parameter<bool>("use_external_yaw", true);
        node_->declare_parameter<double>("gains/rot/x", 1.5);
        node_->declare_parameter<double>("gains/rot/y", 1.5);
        node_->declare_parameter<double>("gains/rot/z", 1.0);
        node_->declare_parameter<double>("gains/ang/x", 0.13);
        node_->declare_parameter<double>("gains/ang/y", 0.13);
        node_->declare_parameter<double>("gains/ang/z", 0.1);
        node_->declare_parameter<double>("gains/kx/x", 5.7);
        node_->declare_parameter<double>("gains/kx/y", 5.7);
        node_->declare_parameter<double>("gains/kx/z", 6.2);
        node_->declare_parameter<double>("gains/kv/x", 3.4);
        node_->declare_parameter<double>("gains/kv/y", 3.4);
        node_->declare_parameter<double>("gains/kv/z", 4.0);
        node_->declare_parameter<double>("corrections/z", 0.0);
        node_->declare_parameter<double>("corrections/r", 0.0);
        node_->declare_parameter<double>("corrections/p", 0.0);
        node_->declare_parameter<double>("so3_control/init_state_x", 0.0);
        node_->declare_parameter<double>("so3_control/init_state_y", 0.0);
        node_->declare_parameter<double>("so3_control/init_state_z", -10000.0);

        node_->get_parameter("quadrotor_name", quadrotor_name_);
        frame_id_ = "/" + quadrotor_name_;

        double mass;
        node_->get_parameter("mass", mass);
        controller_.setMass(mass);

        node_->get_parameter("use_external_yaw", use_external_yaw_);

        node_->get_parameter("gains/rot/x", kR_[0]);
        node_->get_parameter("gains/rot/y", kR_[1]);
        node_->get_parameter("gains/rot/z", kR_[2]);
        node_->get_parameter("gains/ang/x", kOm_[0]);
        node_->get_parameter("gains/ang/y", kOm_[1]);
        node_->get_parameter("gains/ang/z", kOm_[2]);
        node_->get_parameter("gains/kx/x", kx_[0]);
        node_->get_parameter("gains/kx/y", kx_[1]);
        node_->get_parameter("gains/kx/z", kx_[2]);
        node_->get_parameter("gains/kv/x", kv_[0]);
        node_->get_parameter("gains/kv/y", kv_[1]);
        node_->get_parameter("gains/kv/z", kv_[2]);

        node_->get_parameter("corrections/z", corrections_[0]);
        node_->get_parameter("corrections/r", corrections_[1]);
        node_->get_parameter("corrections/p", corrections_[2]);

        node_->get_parameter("so3_control/init_state_x", init_x_);
        node_->get_parameter("so3_control/init_state_y", init_y_);
        node_->get_parameter("so3_control/init_state_z", init_z_);

        so3_command_pub_ = node_->create_publisher<quadrotor_msgs::msg::SO3Command>("so3_cmd", 10);

        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&SO3ControlNode::odom_callback, this, std::placeholders::_1));
        position_cmd_sub_ = node_->create_subscription<quadrotor_msgs::msg::PositionCommand>(
            "position_cmd", 10, std::bind(&SO3ControlNode::position_cmd_callback, this, std::placeholders::_1));
        enable_motors_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "motors", 2, std::bind(&SO3ControlNode::enable_motors_callback, this, std::placeholders::_1));
        corrections_sub_ = node_->create_subscription<quadrotor_msgs::msg::Corrections>(
            "corrections", 10, std::bind(&SO3ControlNode::corrections_callback, this, std::placeholders::_1));
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "imu", 10, std::bind(&SO3ControlNode::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "SO3 Control node has been initialized");
    }

private:
    void publishSO3Command() {
        controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_, kx_, kv_);

        const Eigen::Vector3d &force = controller_.getComputedForce();
        const Eigen::Quaterniond &orientation = controller_.getComputedOrientation();

        auto so3_command = std::make_shared<quadrotor_msgs::msg::SO3Command>();
        so3_command->header.stamp = node_->now();
        so3_command->header.frame_id = frame_id_;
        so3_command->force.x = force(0);
        so3_command->force.y = force(1);
        so3_command->force.z = force(2);
        so3_command->orientation.x = orientation.x();
        so3_command->orientation.y = orientation.y();
        so3_command->orientation.z = orientation.z();
        so3_command->orientation.w = orientation.w();
        for (int i = 0; i < 3; i++) {
            so3_command->kr[i] = kR_[i];
            so3_command->kom[i] = kOm_[i];
        }
        so3_command->aux.current_yaw = current_yaw_;
        so3_command->aux.kf_correction = corrections_[0];
        so3_command->aux.angle_corrections[0] = corrections_[1];
        so3_command->aux.angle_corrections[1] = corrections_[2];
        so3_command->aux.enable_motors = enable_motors_;
        so3_command->aux.use_external_yaw = use_external_yaw_;
        so3_command_pub_->publish(*so3_command);
    }

    void position_cmd_callback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd) {
        des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
        des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
        des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);

        if (cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5) {
            kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
        }
        if (cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5) {
            kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
        }

        des_yaw_ = cmd->yaw;
        des_yaw_dot_ = cmd->yaw_dot;
        position_cmd_updated_ = true;
        position_cmd_init_ = true;

        publishSO3Command();
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom) {
        const Eigen::Vector3d position(odom->pose.pose.position.x,
                                       odom->pose.pose.position.y,
                                       odom->pose.pose.position.z);
        const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                       odom->twist.twist.linear.y,
                                       odom->twist.twist.linear.z);

        tf2::Quaternion quaternion;
        fromMsg(odom->pose.pose.orientation, quaternion);
        current_yaw_ = tf2::impl::getYaw(quaternion);

        controller_.setPosition(position);
        controller_.setVelocity(velocity);

        if (position_cmd_init_) {
            if (!position_cmd_updated_)
                publishSO3Command();
            position_cmd_updated_ = false;
        } else if (init_z_ > -9999.0) {
            des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
            des_vel_ = Eigen::Vector3d(0, 0, 0);
            des_acc_ = Eigen::Vector3d(0, 0, 0);
            publishSO3Command();
        }
    }

    void enable_motors_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data)
            RCLCPP_INFO(node_->get_logger(), "Enabling motors");
        else
            RCLCPP_INFO(node_->get_logger(), "Disabling motors");

        enable_motors_ = msg->data;
    }

    void corrections_callback(const quadrotor_msgs::msg::Corrections::SharedPtr msg) {
        corrections_[0] = msg->kf_correction;
        corrections_[1] = msg->angle_corrections[0];
        corrections_[2] = msg->angle_corrections[1];
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu) {
        const Eigen::Vector3d acc(imu->linear_acceleration.x,
                                  imu->linear_acceleration.y,
                                  imu->linear_acceleration.z);
        controller_.setAcc(acc);
    }

    rclcpp::Node::SharedPtr node_;
    SO3Control controller_;
    rclcpp::Publisher<quadrotor_msgs::msg::SO3Command>::SharedPtr so3_command_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr position_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_motors_sub_;
    rclcpp::Subscription<quadrotor_msgs::msg::Corrections>::SharedPtr corrections_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    bool position_cmd_updated_, position_cmd_init_;
    std::string frame_id_, quadrotor_name_;

    Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
    double des_yaw_, des_yaw_dot_;
    double current_yaw_;
    bool enable_motors_;
    bool use_external_yaw_;
    double kR_[3], kOm_[3], corrections_[3];
    double init_x_, init_y_, init_z_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("so3_control_node");
    auto so3_control_node = std::make_shared<SO3ControlNode>(node);
    spin(node);
    rclcpp::shutdown();
    return 0;
}
