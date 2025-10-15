#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// --- CRITICAL FIX: Ensure C linkage for the pigpio client library ---
// This tells the C++ compiler to link these functions as C functions, 
// resolving the 'not declared in this scope' error.
extern "C" {
#include <pigpiod_if.h> 
}
// -------------------------------------------------------------------

#include <chrono>
#include <cstdlib>

// Define the GPIO pin for the sensor (Placeholder BCM 4)
#define SOIL_MOISTURE_PIN 4 

// Use milliseconds for the timer period
using namespace std::chrono_literals;

class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher() : Node("sensor_publisher_node")
    {
        // 1. Initialize PiGPIO (Connect as a client)
        // NULL for address and NULL for port means connect to localhost:8888
        pigpio_handle_ = gpioConnect(NULL, NULL);
        if (pigpio_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO connection failed with error code: %d. Ensure the 'pigpiod' daemon is running.", pigpio_handle_);
            throw std::runtime_error("PiGPIO initialization failed: Could not connect to daemon.");
        }
        RCLCPP_INFO(this->get_logger(), "PiGPIO client connected successfully (Handle: %d) for sensor reading.", pigpio_handle_);

        // 2. Setup Sensor Pin
        // Use the handle-based function to set the mode
        gpiodSetMode(pigpio_handle_, SOIL_MOISTURE_PIN, PI_INPUT);

        // 3. Create Publisher and Timer
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpi_sensor_data", 10);
        timer_ = this->create_wall_timer(
            500ms, 
            std::bind(&SensorPublisher::publish_sensor_data, this));

        RCLCPP_INFO(this->get_logger(), "SensorPublisher ready, publishing to /rpi_sensor_data...");
    }

    ~SensorPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SensorPublisher. Cleaning up PiGPIO connection.");
        // Use the handle-based terminate function
        gpiodTerminate(pigpio_handle_); 
    }

private:
    void publish_sensor_data()
    {
        // Use the handle-based function to read the pin state
        int digital_reading = gpiodRead(pigpio_handle_, SOIL_MOISTURE_PIN);
        
        // Simulating sensor data based on digital reading for demonstration.
        // The digital reading is either 0 (HIGH/dry) or 1 (LOW/wet) for a typical digital sensor.
        // The analog simulation creates a value between 500 and 1000.
        double simulated_adc_value = 1000.0 - (digital_reading * 500.0) + (std::rand() % 50);

        auto message = std_msgs::msg::Float64();
        message.data = simulated_adc_value;
        
        publisher_->publish(message);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Publishing Sensor Data: %.2f (Pin %d: %d)", 
            message.data, SOIL_MOISTURE_PIN, digital_reading);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    int pigpio_handle_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<SensorPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        // Catches and logs the pigpio connection error if it fails
        RCLCPP_ERROR(rclcpp::get_logger("sensor_publisher_main"), "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
