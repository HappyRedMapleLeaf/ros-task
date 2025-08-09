#ifndef UTIL_HPP
#define UTIL_HPP

#include "rclcpp/rclcpp.hpp"

namespace ros_task {

struct Pose2D {
    double x;
    double y;
    double theta;
};

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