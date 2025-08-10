#include "util.hpp"

ros_task::P_Controller::P_Controller(double kp, double abs_limit) : kp_(kp), abs_limit_(abs_limit) {}
ros_task::P_Controller::P_Controller() : kp_(0.0), abs_limit_(0.0) {}

// very simple proportional controller with absolute limit
double ros_task::P_Controller::update(const double error,
                                      rclcpp::Duration dt) {
    double value = kp_ * error * dt.seconds();

    // Apply absolute limit
    if (value > abs_limit_) {
        return abs_limit_;
    } else if (value < -abs_limit_) {
        return -abs_limit_;
    }
    return value;
}