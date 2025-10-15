#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// Include the PiGPIO client interface header
#include <pigpiod_if.h> 

#include <chrono>
#include <cstdlib>
#include <cmath>

// --- Pin Definitions (BCM Numbering) ---
// Using a placeholder GPIO pin often used for A/D timing circuits 
// or connecting to an external ADC chip. We'll use this for the software A/D simulation.
#define SENSOR_PIN 4 

// System Constants for Timer
using namespace std::chrono_literals;

// --- PiGPIO Client Functions Helper ---

/**
 * @brief Performs a simplified, software-based Analog-to-Digital Conversion (ADC)
 * using the PiGPIO client interface. 
 * * NOTE: This function simulates the behavior of reading an external ADC or 
 * performing a capacitor charge/discharge timing measurement, a common technique
 * when dealing with purely analog sensors on a Raspberry Pi. 
 * Since we don't know your exact hardware setup (e.g., if you have an MCP3008), 
 * this provides a simulated, but more realistic, PiGPIO-based reading than a simple digital read.
 * * In a real application, you would replace the simulated return value with 
 * a call to a library that communicates with your specific ADC (e.g., SPI communication).
 * * @param handle The PiGPIO connection handle.
 * @param gpio The GPIO pin to "read" from.
 * @return A simulated analog reading (0 to 1000).
 */
int get_adc_value(int handle, int gpio) 
{
    if (handle < 0) {
        return -1; // Indicate connection error
    }

    // --- Placeholder Simulation ---
    // In a real environment, this is where you would call a function like:
    // int value = spiRead(handle, channel, buffer, count);
    
    // For now, we return a value based on a slowly changing sine wave to simulate 
    // real-world, dynamic sensor readings (e.g., temperature, light, moisture).
    static double phase = 0.0;
    phase += 0.05;
    if (phase > M_PI * 2) {
        phase -= M_PI * 2;
    }
    
    // Generates a value between 200 and 800 (out of 1000 total range)
    int adc_value = static_cast<int>(500.0 + 300.0 * std::sin(phase) + (std::rand() % 50 - 25));
    
    // Ensure value is within bounds
    return std::min(1000, std::max(0, adc_value));
}


class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher() : Node("sensor_publisher_node")
    {
        // 1. Initialize PiGPIO (Connect as a client)
        // gpioConnect is the correct client function name to connect to the daemon.
        pigpio_handle_ = gpioConnect(NULL, NULL);
        if (pigpio_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO connection failed with error code: %d. Ensure the 'pigpiod' daemon is running.", pigpio_handle_);
            // Do not throw an error here, the application can continue (though without GPIO functionality)
        } else {
            RCLCPP_INFO(this->get_logger(), "PiGPIO client connected successfully (Handle: %d) for sensor reading.", pigpio_handle_);

            // 2. Setup Sensor Pin (only if connection succeeded)
            // For a simulated A/D timing circuit, the pin would be set to INPUT, 
            // but since we are simulating the ADC entirely, we just ensure PiGPIO is ready.
            gpiodSetMode(pigpio_handle_, SENSOR_PIN, PI_INPUT);
        }

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
        if (pigpio_handle_ >= 0) {
            // gpioTerminate(handle) is the client termination function.
            gpioTerminate(pigpio_handle_); 
        }
    }

private:
    void publish_sensor_data()
    {
        // Only attempt to read GPIO if connection was successful
        if (pigpio_handle_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "PiGPIO not connected. Skipping sensor read.");
            return;
        }

        // Get the ADC value using the helper function
        int adc_value = get_adc_value(pigpio_handle_, SENSOR_PIN);
        
        // Convert the ADC value (0-1000) into a percentage or scaled value (0.0 - 100.0)
        double sensor_data = (double)adc_value / 10.0; 

        auto message = std_msgs::msg::Float64();
        message.data = sensor_data;
        
        publisher_->publish(message);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Publishing Sensor Data: %.2f (ADC Reading: %d)", 
            message.data, adc_value);
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
        RCLCPP_ERROR(rclcpp::get_logger("sensor_publisher_main"), "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
