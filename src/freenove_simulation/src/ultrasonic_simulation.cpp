#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

class UltrasonicFilter : public rclcpp::Node {
public:
    UltrasonicFilter() : Node("ultrasonic_filter") {
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/ultrasonic/scan", 10,
            std::bind(&UltrasonicFilter::scanCallback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::Range>("/ultrasonic/range", 10);

        RCLCPP_INFO(this->get_logger(), "simulation of ultrasonic sensor started");
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg) {
        auto range_msg = sensor_msgs::msg::Range();
        
        range_msg.header = scan_msg->header;
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.field_of_view = 0.2618; // ~15 degress in radiants
        range_msg.min_range = scan_msg->range_min;
        range_msg.max_range = scan_msg->range_max;

        float min_dist = scan_msg->range_max;
        bool valid_hit = false;

        for (float r : scan_msg->ranges) {
            if (std::isfinite(r) && r >= scan_msg->range_min && r <= scan_msg->range_max) {
                min_dist = std::min(min_dist, r);
                valid_hit = true;
            }
        }

        range_msg.range = valid_hit ? min_dist : std::numeric_limits<float>::infinity();

        pub_->publish(range_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicFilter>());
    rclcpp::shutdown();
    return 0;
}