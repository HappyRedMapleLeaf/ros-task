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
        P_Controller(double kp);
        double update(const double current, const double target, rclcpp::Duration dt);
        
    private:
        double kp_;
};

} // namespace ros_task

#endif // UTIL_HPP