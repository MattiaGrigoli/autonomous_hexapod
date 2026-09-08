#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>

class HexapodGaitNode : public rclcpp::Node {
public:
    HexapodGaitNode() : Node("hexapod_gait_node"), gait_phase_(0), head_pan_rad_(0.0), head_tilt_rad_(0.0) {
        this->declare_parameter<bool>("sim", false);
        sim = this->get_parameter("sim").as_bool();

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&HexapodGaitNode::cmdVelCallback, this, std::placeholders::_1));

        sub_cmd_head_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/cmd_head", 10, std::bind(&HexapodGaitNode::cmdHeadCallback, this, std::placeholders::_1));

        sub_body_height = this->create_subscription<std_msgs::msg::Float64>("/set_body_height", 10, std::bind(&HexapodGaitNode::set_body_height, this, std::placeholders::_1));
        sub_step_height = this->create_subscription<std_msgs::msg::Float64>("/set_step_height", 10, std::bind(&HexapodGaitNode::set_step_height, this, std::placeholders::_1));

        if(!sim)
        {
            pub_joint_states_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        }

        joint_names_ = {
            "motor0", "motor1",
            "motor16", "motor17", "motor18",
            "motor19", "motor20", "motor21",
            "motor22", "motor23", "motor27",
            "motor9", "motor8", "motor31",
            "motor12", "motor11", "motor10",
            "motor15", "motor14", "motor13"
        };

        body_points_ = {
            {137.1, 189.4, body_height_}, {225.0, 0.0, body_height_}, {137.1, -189.4, body_height_},
            {-137.1, -189.4, body_height_}, {-225.0, 0.0, body_height_}, {-137.1, 189.4, body_height_}
        };

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), std::bind(&HexapodGaitNode::updateGaitCycle, this));
        
    //     pub_gazebo_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    // "/position_controller/commands", 10);

        pub_leg_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/leg_controller/commands", 10);

        
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_cmd_head_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;
    // rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_gazebo_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_leg_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_body_height;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_step_height;
    bool sim;

    std::vector<std::string> joint_names_;
    std::vector<std::vector<double>> body_points_;
    
    double body_height_ = -95.0;
    double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;
    double head_pan_rad_, head_tilt_rad_;
    int gait_phase_;
    int F = 64; // steps for step
    double lift_z = -40.0; //height step

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = -std::clamp(msg->linear.y * 35.0, -35.0, 35.0);
        vy_ = -std::clamp(msg->linear.x * 35.0, -35.0, 35.0);
        wz_ = std::clamp(msg->angular.z * 20.0, -20.0, 20.0);
    }

    void cmdHeadCallback(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        head_pan_rad_ = std::clamp(msg->x, -90.0, 90.0) * M_PI / 180.0;
        head_tilt_rad_ = std::clamp(msg->y, -90.0, 90.0) * M_PI / 180.0;
    }

    void set_body_height(const std_msgs::msg::Float64::SharedPtr msg)
    {
        body_height_ = -msg->data;
    }

    void set_step_height(const std_msgs::msg::Float64::SharedPtr msg)
    {
        lift_z = -msg->data;
    }

    bool coordinateToAngle(double x, double y, double z, double &a_deg, double &b_deg, double &c_deg) {
        double l1 = 33.0, l2 = 90.0, l3 = 110.0;
        
        double a = M_PI_2 - std::atan2(z, y);
        double x_4 = l1 * std::sin(a);
        double x_5 = l1 * std::cos(a);
        
        double l23 = std::sqrt(std::pow(z - x_5, 2) + std::pow(y - x_4, 2) + std::pow(x, 2));
        if (l23 > (l2 + l3) || l23 < std::abs(l2 - l3)) return false;

        double v = std::clamp((l2 * l2 + l23 * l23 - l3 * l3) / (2.0 * l2 * l23), -1.0, 1.0);
        double u = std::clamp((l2 * l2 + l3 * l3 - l23 * l23) / (2.0 * l3 * l2), -1.0, 1.0);

        double b = std::asin(std::clamp(x / l23, -1.0, 1.0)) - std::acos(v);
        double c = M_PI - std::acos(u);

        a_deg = a * 180.0 / M_PI;
        b_deg = b * 180.0 / M_PI;
        c_deg = c * 180.0 / M_PI;
        return true;
    }

    void transformCoordinates(const std::vector<std::vector<double>> &pts, std::vector<std::vector<double>> &leg_pos) {
        static const double angles[6] = {54.0, 0.0, -54.0, -126.0, 180.0, 126.0};
        static const double offsets[6] = {94.0, 85.0, 94.0, 94.0, 85.0, 94.0};

        for (int i = 0; i < 6; ++i) {
            double rad = angles[i] * M_PI / 180.0;
            leg_pos[i][0] = pts[i][0] * std::cos(rad) + pts[i][1] * std::sin(rad) - offsets[i];
            leg_pos[i][1] = -pts[i][0] * std::sin(rad) + pts[i][1] * std::cos(rad);
            leg_pos[i][2] = pts[i][2] - 14.0;
        }
    }

    void updateGaitCycle() {
        std::vector<std::vector<double>> pts = body_points_;

        // check if movement
        bool is_moving = (std::abs(vx_) > 0.1 || std::abs(vy_) > 0.1 || std::abs(wz_) > 0.1);

        if (!is_moving) {
            // standing
            gait_phase_ = 0;
            for (int i = 0; i < 6; ++i) {
                pts[i][2] = body_height_; // all legs on floor
            }
        } else {
            // freenove gait

            gait_phase_ = (gait_phase_ + 1) % F;

            double rad_angle = wz_ * M_PI / 180.0;
            double cos_a = std::cos(rad_angle);
            double sin_a = std::sin(rad_angle);

            std::vector<std::pair<double, double>> xy(6);
            for (int i = 0; i < 6; ++i) {
                double px = body_points_[i][0];
                double py = body_points_[i][1];
                xy[i].first  = ((px * cos_a + py * sin_a - px) + vx_) / F;
                xy[i].second = ((-px * sin_a + py * cos_a - py) + vy_) / F;
            }

            double S = F / 8.0;
            double p = static_cast<double>(gait_phase_) / S;

            for (int i = 0; i < 3; ++i) {
                int leg_a = 2 * i;     // Group A
                int leg_b = 2 * i + 1; // Group B

                // --- A ---
                if (p < 2.0) {
                    pts[leg_a][0] -= 0.5 * p * F * xy[leg_a].first;
                    pts[leg_a][1] -= 0.5 * p * F * xy[leg_a].second;
                    pts[leg_a][2] = body_height_;
                } else if (p < 3.0) {
                    pts[leg_a][0] -= F * xy[leg_a].first;
                    pts[leg_a][1] -= F * xy[leg_a].second;
                    pts[leg_a][2] = body_height_ + (p - 2.0) * lift_z;
                } else if (p < 5.0) {
                    pts[leg_a][0] += (p - 4.0) * F * xy[leg_a].first;
                    pts[leg_a][1] += (p - 4.0) * F * xy[leg_a].second;
                    pts[leg_a][2] = body_height_ + lift_z;
                } else if (p < 6.0) {
                    pts[leg_a][0] += F * xy[leg_a].first;
                    pts[leg_a][1] += F * xy[leg_a].second;
                    pts[leg_a][2] = body_height_ + (6.0 - p) * lift_z;
                } else {
                    pts[leg_a][0] += (4.0 - 0.5 * p) * F * xy[leg_a].first;
                    pts[leg_a][1] += (4.0 - 0.5 * p) * F * xy[leg_a].second;
                    pts[leg_a][2] = body_height_;
                }

                // --- B ---
                if (p < 1.0) {
                    pts[leg_b][0] += p * F * xy[leg_b].first;
                    pts[leg_b][1] += p * F * xy[leg_b].second;
                    pts[leg_b][2] = body_height_ + lift_z;
                } else if (p < 2.0) {
                    pts[leg_b][0] += F * xy[leg_b].first;
                    pts[leg_b][1] += F * xy[leg_b].second;
                    pts[leg_b][2] = body_height_ + (2.0 - p) * lift_z;
                } else if (p < 6.0) {
                    pts[leg_b][0] += (2.0 - 0.5 * p) * F * xy[leg_b].first;
                    pts[leg_b][1] += (2.0 - 0.5 * p) * F * xy[leg_b].second;
                    pts[leg_b][2] = body_height_;
                } else if (p < 7.0) {
                    pts[leg_b][0] -= F * xy[leg_b].first;
                    pts[leg_b][1] -= F * xy[leg_b].second;
                    pts[leg_b][2] = body_height_ + (p - 6.0) * lift_z;
                } else {
                    pts[leg_b][0] += (p - 8.0) * F * xy[leg_b].first;
                    pts[leg_b][1] += (p - 8.0) * F * xy[leg_b].second;
                    pts[leg_b][2] = body_height_ + lift_z;
                }
            }
        }

        // transformation and IK
        std::vector<std::vector<double>> leg_positions(6, std::vector<double>(3, 0.0));
        transformCoordinates(pts, leg_positions);

        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.name = joint_names_;

        joint_state.position.push_back(head_pan_rad_);
        joint_state.position.push_back(head_tilt_rad_);

        for (int i = 0; i < 6; ++i) {
            double a, b, c;
            coordinateToAngle(-leg_positions[i][2], leg_positions[i][0], leg_positions[i][1], a, b, c);

            double coxa_rad  = (a - 90.0) * M_PI / 180.0;
            double femur_rad = b * M_PI / 180.0;
            double tibia_rad = c * M_PI / 180.0;

            if (i < 3) {
                femur_rad = -femur_rad;
                tibia_rad = -tibia_rad; 
            }

            joint_state.position.push_back(coxa_rad);
            joint_state.position.push_back(femur_rad);
            joint_state.position.push_back(tibia_rad);
        }

        if(!sim)
        {
            pub_joint_states_->publish(joint_state);
        }

        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data = joint_state.position;

        // pub_gazebo_cmd_->publish(cmd_msg);
        pub_leg_cmd_->publish(cmd_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HexapodGaitNode>());
    rclcpp::shutdown();
    return 0;
}