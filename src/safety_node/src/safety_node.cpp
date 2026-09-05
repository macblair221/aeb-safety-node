#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <cmath>
#include <limits>

using std::placeholders::_1;

class SafetyNode : public rclcpp::Node
{

public:

    SafetyNode() : Node("safety_node"){
        RCLCPP_INFO(this->get_logger(), "Safety Node Started");

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10,
            std::bind(&SafetyNode::odom_callback, this, _1));

        this->declare_parameter("ttc_threshold", 1.0);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SafetyNode::scan_callback, this, _1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
    }



private:

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();

        if (odom_received_ && !braking_ &&
            (this->now() - last_odom_time_).seconds() > 0.5) {
            braking_ = true;
            RCLCPP_WARN(this->get_logger(), "BRAKE - odometry stale");
        }

        double min_ttc = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double range = msg->ranges[i];

            if (!std::isfinite(range) ||
                range < msg->range_min || range > msg->range_max) {
                continue;
            }

            // How fast this particular beam's distance is shrinking.
            // A beam pointing straight ahead closes at full speed;
            // one pointing sideways closes at zero.
            double angle = msg->angle_min + i * msg->angle_increment;
            double closing_rate = speed_ * std::cos(angle);

            // Beam not closing (stationary, reversing, or pointing away)
            // means we will never reach it along this ray.
            if (closing_rate <= 0.0) {
                continue;
            }

            double ttc = range / closing_rate;
            if (ttc < min_ttc) {
                min_ttc = ttc;
            }
        }

        if (min_ttc < ttc_threshold_ && !braking_) {
            braking_ = true;
            RCLCPP_WARN(this->get_logger(),
                        "BRAKE - time to collision %.2f s", min_ttc);
        }

        if (braking_) {
            auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
            drive_msg.header.stamp = this->now();
            drive_msg.drive.speed = 0.0;
            drive_pub_->publish(drive_msg);
        }

        RCLCPP_INFO(this->get_logger(), "min TTC %.2f s", min_ttc);
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        speed_ = msg->twist.twist.linear.x;
        last_odom_time_ = this->now();
        odom_received_ = true;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double speed_ = 0.0;
    bool braking_ = false;
    bool odom_received_ = false;
    rclcpp::Time last_odom_time_;
    double ttc_threshold_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyNode>());
    rclcpp::shutdown();
    return 0;
}