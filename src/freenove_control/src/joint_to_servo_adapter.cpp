#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

struct ServoConfig {
    int channel;
    double offset_deg;
    int direction;
    double min_deg;
    double max_deg;
};

class JointToServoAdapter : public rclcpp::Node {
public:
    JointToServoAdapter() : Node("joint_to_servo_adapter") {
        sub_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointToServoAdapter::jointStateCallback, this, std::placeholders::_1));

        // Topic in which we send degrees to the servomotors/driver I2C
        pub_servo_angles_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/servo_cmd_deg", 10);

        loadParameters();
        RCLCPP_INFO(this->get_logger(), "Node Joint to Servo Adapter launched.");
    }

private:
    void loadParametersManual() {
        // load configuration map
        // example of manual fallback if yaml parameters are not passed
        // servo_map_["rf_hip_joint"]  = {0, 90.0,  1, 0.0, 180.0};
        // servo_map_["rf_flex_joint"] = {1, 90.0, -1, 0.0, 180.0};
        // servo_map_["rf_knee_joint"] = {2, 90.0,  1, 0.0, 180.0};
    }

    void loadParameters() {
        // get all parameters starting with servos"servos"
        auto param_list = this->list_parameters({}, 0);

        // 2. ESTRAZIONE DIRETTA DAGLI OVERRIDES YAML
        const auto & overrides = this->get_node_parameters_interface()->get_parameter_overrides();
        const std::string prefix = "servos.";

        for (const auto & [param_name, param_val] : overrides) {
            // check if starts with "servos"
            if (param_name.rfind(prefix, 0) == 0) {
                std::string joint_name = param_name.substr(prefix.length());

                if (param_val.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                    std::vector<double> vec = param_val.get<std::vector<double>>();

                    if (vec.size() == 5) {
                        servo_map_[joint_name] = {
                            static_cast<int>(vec[0]), // channel
                            vec[1],                   // offset_deg
                            static_cast<int>(vec[2]), // direction
                            vec[3],                   // min_deg
                            vec[4]                    // max_deg
                        };
                        RCLCPP_INFO(this->get_logger(), " -> Mapped %s on channel I2C %d", joint_name.c_str(), static_cast<int>(vec[0]));
                    } else {
                        RCLCPP_WARN(this->get_logger(), "Parameter '%s' ignored: expected 5 values", param_name.c_str());
                    }
                }
            }
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        auto cmd_array = std_msgs::msg::Float64MultiArray();
        cmd_array.data.resize(20, 90.0); // default 90 degrees for the 20 servomotors

        for (size_t i = 0; i < msg->name.size(); ++i) {
            const std::string & joint_name = msg->name[i];
            
            if (servo_map_.find(joint_name) != servo_map_.end()) {
                const auto & cfg = servo_map_[joint_name];
                
                double rad = msg->position[i];
                double deg = cfg.offset_deg + (rad * (180.0 / M_PI) * cfg.direction);

                // security clamp for mechanical limits
                deg = std::clamp(deg, cfg.min_deg, cfg.max_deg);

                cmd_array.data[cfg.channel] = deg;
            }
        }

        pub_servo_angles_->publish(cmd_array);
    }

    std::map<std::string, ServoConfig> servo_map_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_servo_angles_;
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointToServoAdapter>());
    rclcpp::shutdown();
    return 0;
}