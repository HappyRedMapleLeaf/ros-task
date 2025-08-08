#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "util.hpp"

using namespace std::chrono_literals;

#define DEFAULT_MSG_BUF_SIZE    10
#define LOOP_PERIOD             100ms

class RobotController : public rclcpp::Node {
    public:
        RobotController();

    private:
        void main_timer_callback();
        void odom_rx_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void target_rx_callback(const geometry_msgs::msg::Pose::SharedPtr msg);

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr    target_subscription_;

        rclcpp::TimerBase::SharedPtr    main_timer_;
        rclcpp::Time                    prev_loop_time_;

        // data for current and target pose. use own types instead of message structs
        ros_task::Pose current_pose_;
        ros_task::Pose target_pose_;
};

int main(int argc, char * argv[]);

#endif // CONTROLLER_HPP