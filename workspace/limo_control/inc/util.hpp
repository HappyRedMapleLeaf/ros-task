#ifndef UTIL_HPP
#define UTIL_HPP

#include "rclcpp/rclcpp.hpp"

namespace ros_task {

// Simple 2D pose structure
struct Pose2D {
    double x;
    double y;
    double theta;
};

// Proportional controller with absolute limit
// Can easily be extended to include integral or derivative terms if needed
class P_Controller {
    public:
        P_Controller();
        P_Controller(double kp, double abs_limit);
        double update(const double error, rclcpp::Duration dt);
        
    private:
        double kp_;
        double abs_limit_;
};

} // namespace ros_task

#endif // UTIL_HPP