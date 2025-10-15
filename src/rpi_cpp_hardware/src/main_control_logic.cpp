#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>

// System Constants
#define DISTANCE_MIN_CM 15.0 // Obstacle detection threshold (15 cm)
#define SOIL_WET_LEVEL  500  // Example analog reading threshold

class ControlLogic : public rclcpp::Node
{
public:
    ControlLogic() : Node("main_control_node")
    {
        // Initialize state variables
        last_distances_["us1"] = 999.0;
        last_distances_["us4"] = 999.0;
        last_soil_moisture_ = 1000;

        // 1. Create Subscribers
        sub_us1_ = this->create_subscription<std_msgs::msg::Float64>(
            "distance_us1", 10, std::bind(&ControlLogic::us1_callback, this, std::placeholders::_1));
        sub_us4_ = this->create_subscription<std_msgs::msg::Float64>(
            "distance_us4", 10, std::bind(&ControlLogic::us4_callback, this, std::placeholders::_1));
        sub_soil_ = this->create_subscription<std_msgs::msg::Int32>(
            "soil_moisture", 10, std::bind(&ControlLogic::soil_callback, this, std::placeholders::_1));

        // 2. Create Publisher for motor commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 3. Create Decision Timer (runs the core application logic)
        decision_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&ControlLogic::decision_loop, this));

        RCLCPP_INFO(this->get_logger(), "ControlLogic ready, monitoring sensors and publishing to /cmd_vel.");
    }

private:
    // --- Sensor Callbacks (Update State) ---
    void us1_callback(const std_msgs::msg::Float64::SharedPtr msg) { last_distances_["us1"] = msg->data; }
    void us4_callback(const std_msgs::msg::Float64::SharedPtr msg) { last_distances_["us4"] = msg->data; }
    void soil_callback(const std_msgs::msg::Int32::SharedPtr msg) { last_soil_moisture_ = msg->data; }

    // --- Core Decision Logic (Runs on Timer) ---
    void decision_loop()
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        bool obstacle_detected = false;

        // 1. Check Front Obstacle (US1)
        if (last_distances_["us1"] < DISTANCE_MIN_CM && last_distances_["us1"] > 0) {
            // Obstacle in front: Stop
            RCLCPP_WARN(this->get_logger(), "FRONT OBSTACLE at %.1f cm! STOPPING.", last_distances_["us1"]);
            twist_msg.linear.x = 0.0;
            twist_msg.angular.z = 0.0;
            obstacle_detected = true;
        }

        // 2. Check Rear Obstacle (US4) - Important for reversing logic
        if (last_distances_["us4"] < DISTANCE_MIN_CM && last_distances_["us4"] > 0) {
            RCLCPP_INFO(this->get_logger(), "REAR OBSTACLE at %.1f cm. Reversing restricted.", last_distances_["us4"]);
            // In a more complex system, this would prevent a reverse command.
        }

        // 3. Check Soil Moisture
        if (last_soil_moisture_ < SOIL_WET_LEVEL) {
            RCLCPP_INFO(this->get_logger(), "ACTION: Soil is dry (%d). Watering needed or moving to recharge.", last_soil_moisture_);
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "STATUS: Soil moisture OK (%d).", last_soil_moisture_);
        }

        // 4. Default Motion (If no obstacle)
        if (!obstacle_detected) {
            // Default: Move forward at a slow speed
            twist_msg.linear.x = 0.3; // 30% speed
            twist_msg.angular.z = 0.0;
        }

        // Publish the determined command
        cmd_vel_pub_->publish(twist_msg);
    }

    // State
    std::map<std::string, double> last_distances_;
    int last_soil_moisture_;

    // ROS 2 Members
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_us1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_us4_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_soil_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr decision_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlLogic>());
    rclcpp::shutdown();
    return 0;
}
