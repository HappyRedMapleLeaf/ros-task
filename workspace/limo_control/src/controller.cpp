#include <chrono>

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "controller.hpp"

using namespace std::chrono_literals;

#define DEFAULT_MSG_BUF_SIZE     10
#define LOOP_PERIOD              200ms

// requirements: within 5cm (0.05m) straight line error and 0.1 rad angular error
// arbitrarily decrease threshold to be safe
#define DISTANCE_ERROR_THRESHOLD 0.05 / 4.0
#define ANGULAR_ERROR_THRESHOLD  0.1 / 4.0

namespace {
    double wrap_angle(double angle) {
        // normalize angle to [-pi, pi]
        angle = fmod(angle, 2 * M_PI);

        if (angle > M_PI) {
            return angle - 2 * M_PI;
        } else if (angle < -M_PI) {
            return angle + 2 * M_PI;
        }
        return angle;
    }
}

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

    // initialize controllers with tuned values and appropriate limits
    this->linear_velocity_controller_ = ros_task::P_Controller(3.0, 0.4);
    this->angular_velocity_controller_ = ros_task::P_Controller(6.0, 0.5);

    this->prev_loop_time_ = this->now();
}

void RobotController::main_timer_callback() {
    rclcpp::Duration elapsed_time = this->now() - this->prev_loop_time_;
    this->prev_loop_time_ = this->now();

    RCLCPP_INFO(
        this->get_logger(),
        "Odom: X: %.2f, Y: %.2f, Theta: %.2f",
        this->current_pose_.x, this->current_pose_.y, this->current_pose_.theta
    );

    /**
     * Main control logic:
     * 1. If both position and angle are within threshold, do nothing.
     * 2. If the robot is close enough to the target position but not the target angle,
     *    only rotate to face the target angle.
     * 3. If the robot is not close enough to the target position, ignore the target angle and:
     *    a) first rotate to face the target position (within some threshold), then
     *    b) move straight towards it, while still making angle adjustments.
     */

    // command to send
    geometry_msgs::msg::Twist vel_cmd;

    // determine if we are close enough to the target position
    double euclidean_dist = hypot(
        this->target_pose_.x - this->current_pose_.x,
        this->target_pose_.y - this->current_pose_.y
    );

    if (fabs(euclidean_dist) < DISTANCE_ERROR_THRESHOLD) {
        if (fabs(wrap_angle(this->target_pose_.theta - this->current_pose_.theta)) < ANGULAR_ERROR_THRESHOLD) {
            // 1. do nothing
            RCLCPP_INFO(this->get_logger(), "Reached target");
        } else {
            // 2. rotate to target angle
            double angular_error = wrap_angle(this->target_pose_.theta - this->current_pose_.theta);

            vel_cmd.angular.z = angular_velocity_controller_.update(angular_error, elapsed_time);

            RCLCPP_INFO(
                this->get_logger(),
                "Final rotation. Current angle: %.2f, Target angle: %.2f, Angular Vel: %.2f",
                this->current_pose_.theta, this->target_pose_.theta, vel_cmd.angular.z
            );
        }
    } else {
        // 2. Determine angle to face target; ignore actual target angle
        double target_angle = atan2(
            this->target_pose_.y - this->current_pose_.y,
            this->target_pose_.x - this->current_pose_.x
        );

        double angular_error = wrap_angle(target_angle - this->current_pose_.theta);

        // for both 3a and 3b, adjust angular velocity
        vel_cmd.angular.z = angular_velocity_controller_.update(angular_error, elapsed_time);

        if (fabs(angular_error) < ANGULAR_ERROR_THRESHOLD) {
            // 3b. move straight towards target. angular_error will still be used for minor adjustments

            // Use distance error projected onto the robot's orientation. Probably not necessary but
            // might help with some edge cases (eg. target is slightly off to the side)
            double linear_error = (this->target_pose_.x - this->current_pose_.x) * cos(this->current_pose_.theta) +
                        (this->target_pose_.y - this->current_pose_.y) * sin(this->current_pose_.theta);

            vel_cmd.linear.x = linear_velocity_controller_.update(linear_error, elapsed_time);

            RCLCPP_INFO(
                this->get_logger(),
                "Moving straight. X_o: %.2f, Y_o: %.2f, X_g: %.2f, Y_g: %.2f, Linear Vel: %.2f",
                this->current_pose_.x, this->current_pose_.y, this->target_pose_.x, this->target_pose_.y, vel_cmd.linear.x
            );
        } else {
            // 3a. rotate to face target position

            RCLCPP_INFO(
                this->get_logger(),
                "Rotating to face target. Actual angle: %.2f, Target angle: %.2f, Angular Vel: %.2f",
                this->current_pose_.theta, target_angle, vel_cmd.angular.z
            );
        }
    }
    
    this->cmd_vel_publisher_->publish(vel_cmd);
}

void RobotController::odom_rx_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // update current pose
    this->current_pose_.x = msg->pose.pose.position.x;
    this->current_pose_.y = msg->pose.pose.position.y;

    // Use tf2 utility functions to convert quaternion to yaw
    tf2::Quaternion converted_quat;
    tf2::fromMsg(msg->pose.pose.orientation, converted_quat);
    this->current_pose_.theta = tf2::getYaw(converted_quat);
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