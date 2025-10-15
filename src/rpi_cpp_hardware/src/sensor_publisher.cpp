#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

// Include the PiGPIO client interface header
#include <pigpiod_if.h> 

#include <chrono>
#include <cstdlib>
#include <cmath>

// --- Pin Definitions (BCM Numbering) ---
#define SENSOR_PIN 4 

// System Constants for Timer
using namespace std::chrono_literals;

// --- PiGPIO Client Functions Helper ---

/**
 * @brief Placeholder function for reading an external Analog-to-Digital Converter (ADC).
 * * NOTE: Since the Raspberry Pi GPIOs are digital, this function currently provides
 * simulated data. To read a real analog sensor (like moisture or light), you must
 * implement communication with an external ADC chip (e.g., MCP3008 via SPI or I2C).
 * * @param handle The PiGPIO connection handle.
 * @param _gpio The GPIO pin number (marked as unused for simulation purposes).
 * @return A simulated analog reading (0 to 1000). Returns -1 on error.
 */
int get_adc_value(int handle, int _gpio) // Renamed 'gpio' to '_gpio' to suppress the unused parameter warning
{
    if (handle < 0) {
        return -1; // Indicate connection error
    }

    // =========================================================================
    // --- REAL ADC IMPLEMENTATION GOES HERE ---
    // Example (if using a pre-configured SPI ADC):
    // int adc_channel = 0; // Use appropriate channel
    // int real_value = spiRead(handle, spi_channel_handle, buffer, count);
    
    // For now, we continue using the simulation below:
    // =========================================================================
    
    // Simulation logic: Slowly changing sine wave with minor random noise
    static double phase = 0.0;
    static const int MAX_READING = 1000;
    static const int MIN_READING = 0;

    phase += 0.05;
    if (phase > M_PI * 2) {
        phase -= M_PI * 2;
    }
    
    // Generates a value between 200 and 800
    double simulated_base = 500.0 + 300.0 * std::sin(phase);
    double simulated_noise = (std::rand() % 50 - 25);
    
    int adc_value = static_cast<int>(simulated_base + simulated_noise);
    
    // Ensure the value is within the 0-1000 range
    return std::min(MAX_READING, std::max(MIN_READING, adc_value));
}


class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher() : Node("sensor_publisher_node")
    {
        // 1. Initialize PiGPIO (Connect as a client to the running daemon)
        pigpio_handle_ = gpioConnect(NULL, NULL);
        if (pigpio_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO connection failed with error code: %d. Ensure the 'pigpiod' daemon is running.", pigpio_handle_);
        } else {
            RCLCPP_INFO(this->get_logger(), "PiGPIO client connected successfully (Handle: %d) for sensor reading.", pigpio_handle_);

            // 2. Setup Sensor Pin 
            // NOTE: For ADC, this pin is often unused or acts as a chip select (CS).
            // We set it to input here for compatibility with a simple digital sensor, 
            // but it's not strictly necessary for most ADC setups.
            gpioSetMode(pigpio_handle_, SENSOR_PIN, PI_INPUT);
        }

        // 3. Create Publisher and Timer
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("rpi_sensor_data", 10);
        timer_ = this->create_wall_timer(
            500ms, // Publish every 0.5 seconds
            std::bind(&SensorPublisher::publish_sensor_data, this));

        RCLCPP_INFO(this->get_logger(), "SensorPublisher ready, publishing to /rpi_sensor_data...");
    }

    ~SensorPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SensorPublisher. Cleaning up PiGPIO connection.");
        if (pigpio_handle_ >= 0) {
            // Disconnect the client from the pigpiod daemon
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

        // Get the ADC value (simulated or real)
        int adc_value = get_adc_value(pigpio_handle_, SENSOR_PIN);
        
        if (adc_value < 0) {
             RCLCPP_ERROR(this->get_logger(), "Failed to read sensor value (Error code: %d).", adc_value);
             return;
        }

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
