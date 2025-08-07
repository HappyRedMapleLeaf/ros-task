#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/geometry_msgs/msg/twist.hpp"
#include "nav_msgs/nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

#define DEFAULT_MSG_BUF_SIZE    10
#define LOOP_PERIOD             100ms

class RobotController : public rclcpp::Node {
    public:
        RobotController() : 
        Node("demo_robot_controller") {
            this->cmd_vel_publisher_ = this->create_publisher<std_msgs::msg::String>("cmd_vel", 10);

            this->main_timer_ = this->create_wall_timer(
                LOOP_PERIOD, std::bind(&RobotController::main_timer_callback, this)
            );

            this->odom_subscription_ = this->create_subscription<navmsgs::msg::Odometry>(
                "odom", DEFAULT_MSG_BUF_SIZE, std::bind(&RobotController::odom_rx_callback, this)
            );

            this->target_subscription_ = this->create_subscription<geometrymsgs::msg::Pose>(
                "target", DEFAULT_MSG_BUF_SIZE, std::bind(&RobotController::target_rx_callback, this)
            );

            this->prev_loop_time_ = this->now();
        }

    private:
        void main_timer_callback() {
            rclcpp::Time elapsed_time = this->now() - prev_loop_time_;

            // do p controller and publish to cmd_vel

            prev_loop_time_ = this->now();
        }

        void odom_rx_callback() {
            // update current pose
        }

        void target_rx_callback() {
            // update target pose
        }

        rclcpp::Publisher<geometrymsgs::msg::Twist>::SharedPtr      cmd_vel_publisher_;
        rclcpp::Subscription<navmsgs::msg::Odometry>::SharedPtr     odom_subscription_;
        rclcpp::Subscription<geometrymsgs::msg::Pose>::SharedPtr    target_subscription_;

        rclcpp::TimerBase::SharedPtr    main_timer_;
        rclcpp::Time                    prev_loop_time_;

        // data for curent and target pose. use own types instead of message structs
  };

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}