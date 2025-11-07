#include "rclcpp/rclcpp.hpp"
#include "rpi_cpp_hardware/msg/rpi_sensor_data.hpp"
#include "std_msgs/msg/string.hpp"
#include <iomanip>

using namespace std::chrono_literals;

// System Constants (copied from original script for context)
#define DISTANCE_MIN    10.0 // cm
#define SOIL_WET_LEVEL  500  // Example analog reading threshold

class MainControlLogic : public rclcpp::Node
{
public:
    MainControlLogic()
    : Node("main_control_logic")
    {
        RCLCPP_INFO(this->get_logger(), "Main Control Logic node started.");

        // 1. Create a Subscriber for the Sensor Data
        // The topic is "rpi_sensor_data" and uses the generated RpiSensorData message
        sensor_subscription_ = this->create_subscription<rpi_cpp_hardware::msg::RpiSensorData>(
            "rpi_sensor_data", 
            10, 
            std::bind(&MainControlLogic::sensor_data_callback, this, std::placeholders::_1)
        );

        // 2. Create a Publisher for High-Level Robot Commands (e.g., to control motors)
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("robot_command", 10);
        
        // Timer for publishing simple "keep moving" command if no issues
        publish_timer_ = this->create_wall_timer(
            1s, std::bind(&MainControlLogic::publish_default_command, this));
    }

private:
    rclcpp::Subscription<rpi_cpp_hardware::msg::RpiSensorData>::SharedPtr sensor_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    void sensor_data_callback(const rpi_cpp_hardware::msg::RpiSensorData::SharedPtr msg)
    {
        // Get the relevant data from the incoming ROS message
        double front_distance = msg->us4_distance_cm;
        double rear_distance = msg->us1_distance_cm;
        int soil_moisture = msg->soil_moisture_adc;
        double air_temp = msg->air_temperature_celsius;

        RCLCPP_INFO(this->get_logger(), "--- Received Status Report (Seq: %d) ---", msg->header.stamp.sec);
        RCLCPP_INFO(this->get_logger(), " | Front Distance (US1): %.2f cm", front_distance);
        RCLCPP_INFO(this->get_logger(), " | Rear Distance (US4): %.2f cm", rear_distance);
        RCLCPP_INFO(this->get_logger(), " | Soil Moisture: %d", soil_moisture);
        RCLCPP_INFO(this->get_logger(), " | Air Temperature: %.2f C", air_temp);

        // --- Decision Logic: Obstacle Avoidance ---
        if (front_distance < DISTANCE_MIN && front_distance > 0) {
            RCLCPP_WARN(this->get_logger(), " | ACTION: Front obstacle detected! Sending STOP and REVERSE command.");
            publish_command("REVERSE");
            return; // Stop processing and wait for next sensor read after command
        }

        if (rear_distance < DISTANCE_MIN && rear_distance > 0) {
            RCLCPP_WARN(this->get_logger(), " | WARNING: Object close behind! Reversing motion restricted.");
            // We could send a command here to prevent reverse motion if the robot was trying to stop/reverse
            publish_command("FORWARD");

        }

        // --- Decision Logic: Soil Check ---
        if (soil_moisture < SOIL_WET_LEVEL) {
            RCLCPP_INFO(this->get_logger(), " | ACTION: Soil is dry. Watering needed. Pausing movement.");
            publish_command("WATER");
        } else {
            RCLCPP_INFO(this->get_logger(), " | STATUS: Path clear and moisture OK.");
        }
    }

    // Publishes a simple string command for demonstration
    void publish_command(const std::string& command)
    {
        auto cmd_msg = std_msgs::msg::String();
        cmd_msg.data = command;
        command_publisher_->publish(cmd_msg);
    }
    
    // Default action to keep the robot moving if no immediate issues
    void publish_default_command()
    {
        // If we were commanded to reverse or water recently, we would delay this.
        // For simplicity, we just keep sending the forward command.
        // A more complex node would manage state (moving, stopped, reversing, watering).
        publish_command("FORWARD");
    }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainControlLogic>());
    rclcpp::shutdown();
    return 0;
}
