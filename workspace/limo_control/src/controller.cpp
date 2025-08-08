#include "controller.hpp"

RobotController::RobotController() : Node("demo_robot_controller") {
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    this->odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",DEFAULT_MSG_BUF_SIZE,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_rx_callback(msg); }
    );

    this->target_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
        "target", DEFAULT_MSG_BUF_SIZE,
        [this](const geometry_msgs::msg::Pose::SharedPtr msg) { this->target_rx_callback(msg); }
    );

    this->main_timer_ = this->create_wall_timer(
        LOOP_PERIOD, std::bind(&RobotController::main_timer_callback, this)
    );

    this->prev_loop_time_ = this->now();
}

void RobotController::main_timer_callback() {
    rclcpp::Duration elapsed_time = this->now() - this->prev_loop_time_;

    // do p controller and publish to cmd_vel
    geometry_msgs::msg::Twist test_msg;
    test_msg.linear.x = 0.1; // relative to the robot's orientation
    test_msg.angular.z = 0.1; // CCW from top view
    // the other directions don't do anything.
    
    this->cmd_vel_publisher_->publish(test_msg);

    RCLCPP_INFO(this->get_logger(), "adsasdasds");

    this->prev_loop_time_ = this->now();
}

void RobotController::odom_rx_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // update current pose
}

void RobotController::target_rx_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // update target pose
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}