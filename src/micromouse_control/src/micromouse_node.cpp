#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"

class Micromouse : public rclcpp::Node
{
public:
    Micromouse() : Node("micromouse_node")
    {
        // Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 5 IR sensor subscribers
        ir_front_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ir_front", 10, std::bind(&Micromouse::ir_front_callback, this, std::placeholders::_1));
        ir_front_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ir_front_left", 10, std::bind(&Micromouse::ir_front_left_callback, this, std::placeholders::_1));
        ir_front_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ir_front_right", 10, std::bind(&Micromouse::ir_front_right_callback, this, std::placeholders::_1));
        ir_left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ir_left", 10, std::bind(&Micromouse::ir_left_callback, this, std::placeholders::_1));
        ir_right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "ir_right", 10, std::bind(&Micromouse::ir_right_callback, this, std::placeholders::_1));

        // Timer to move robot
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Micromouse::move, this));
    }

private:
    // Separate callbacks for each sensor
    void ir_front_callback(const sensor_msgs::msg::Range::SharedPtr msg) { front_dist_ = msg->range; }
    void ir_front_left_callback(const sensor_msgs::msg::Range::SharedPtr msg) { front_left_dist_ = msg->range; }
    void ir_front_right_callback(const sensor_msgs::msg::Range::SharedPtr msg) { front_right_dist_ = msg->range; }
    void ir_left_callback(const sensor_msgs::msg::Range::SharedPtr msg) { left_dist_ = msg->range; }
    void ir_right_callback(const sensor_msgs::msg::Range::SharedPtr msg) { right_dist_ = msg->range; }

    void move()
    {
        geometry_msgs::msg::Twist cmd;

        // Simple wall-following logic
        if(front_dist_ > 0.1)
            cmd.linear.x = 0.05;
        else
            cmd.angular.z = -0.5; // turn right

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir_front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir_front_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir_front_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr ir_right_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Sensor distances
    float front_dist_ = 1.0;
    float front_left_dist_ = 1.0;
    float front_right_dist_ = 1.0;
    float left_dist_ = 1.0;
    float right_dist_ = 1.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Micromouse>());
    rclcpp::shutdown();
    return 0;
}

