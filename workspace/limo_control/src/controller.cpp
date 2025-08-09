#include "controller.hpp"
#include "tf2/utils.hpp"

RobotController::RobotController() : Node("demo_robot_controller") {
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    this->odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",DEFAULT_MSG_BUF_SIZE,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) { this->odom_rx_callback(msg); }
    );

    this->target_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
        "target", DEFAULT_MSG_BUF_SIZE,
        [this](const geometry_msgs::msg::Pose2D::SharedPtr msg) { this->target_rx_callback(msg); }
    );

    this->main_timer_ = this->create_wall_timer(
        LOOP_PERIOD, std::bind(&RobotController::main_timer_callback, this)
    );

    this->prev_loop_time_ = this->now();
}

void RobotController::main_timer_callback() {
    rclcpp::Duration elapsed_time = this->now() - this->prev_loop_time_;

    // do p controller and publish to cmd_vel
    // the linear velocity error is distance to target when target is projected onto the robot's orientation
    // angular velocity error depends on left/right; not 360 degrees


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
    this->current_pose_.x = msg->pose.pose.position.x;
    this->current_pose_.y = msg->pose.pose.position.y;
    this->current_pose_.theta = tf2::getYaw(msg->pose.pose.orientation);
}

void RobotController::target_rx_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    // update local target pose based on received message
    this->target_pose_.x = msg->x;
    this->target_pose_.y = msg->y;
    this->target_pose_.theta = msg->theta;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}