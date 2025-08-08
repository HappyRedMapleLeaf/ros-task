#include "util.hpp"

ros_task::P_Controller::P_Controller(double kp) : kp_(kp) {}

double ros_task::P_Controller::update(const double current, const double target,
                                      rclcpp::Duration dt) {
    double error = target - current;
    return kp_ * error * dt.seconds();
}