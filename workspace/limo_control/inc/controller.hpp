#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "util.hpp"

class RobotController : public rclcpp::Node {
    public:
        RobotController();

    private:
        // main control loop
        void main_timer_callback();

        // callbacks for subscription message reception
        void odom_rx_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void target_rx_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg);

        // publishers and subscribers
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr     cmd_vel_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr target_subscription_;

        // more accurate timing for control loop
        rclcpp::TimerBase::SharedPtr main_timer_;
        rclcpp::Time                 prev_loop_time_;

        // data for current and target pose. use own types instead of message structs
        ros_task::Pose2D current_pose_;
        ros_task::Pose2D target_pose_;

        // Controllers. Just one input/output per controller for now
        ros_task::P_Controller linear_velocity_controller_;
        ros_task::P_Controller angular_velocity_controller_;
};

int main(int argc, char * argv[]);

#endif // CONTROLLER_HPP