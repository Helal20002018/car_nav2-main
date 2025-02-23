#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

class RPMPublisher : public rclcpp::Node {
public:
    RPMPublisher() : Node("rpm_publisher") {
        this->declare_parameter("wheelbase", 0.5);
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("max_rpm", 150.0);
        
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_rpm_ = this->get_parameter("max_rpm").as_double();

        left_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_rpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&RPMPublisher::compute_rpm, this, std::placeholders::_1));
    }

private:
    void compute_rpm(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        double left_wheel_velocity = linear_velocity - (angular_velocity * wheelbase_ / 2.0);
        double right_wheel_velocity = linear_velocity + (angular_velocity * wheelbase_ / 2.0);

        double left_rpm = (left_wheel_velocity / (2 * M_PI * wheel_radius_)) * 60.0;
        double right_rpm = (right_wheel_velocity / (2 * M_PI * wheel_radius_)) * 60.0;
        
        left_rpm = std::clamp(left_rpm, -max_rpm_, max_rpm_);
        right_rpm = std::clamp(right_rpm, -max_rpm_, max_rpm_);

        auto left_msg = std_msgs::msg::Float64();
        auto right_msg = std_msgs::msg::Float64();
        left_msg.data = left_rpm;
        right_msg.data = right_rpm;

        left_rpm_pub_->publish(left_msg);
        right_rpm_pub_->publish(right_msg);
    }

    double wheelbase_;
    double wheel_radius_;
    double max_rpm_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_rpm_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_rpm_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPMPublisher>());
    rclcpp::shutdown();
    return 0;
}
