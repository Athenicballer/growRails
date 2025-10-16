#include "rclcpp/rclcpp.hpp"
#include "rpi_cpp_hardware/msg/rpi_sensor_data.hpp" // Custom message header
#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <pigpiod_if2.h>
#include <iomanip>
#include <cmath> 
#include <chrono>

using namespace std::chrono_literals;

// --- Pin and I2C Definitions from original script ---
// Ultrasonic Sensor Pins (HC-SR04)
#define US1_TRIG_PIN    5 
#define US1_ECHO_PIN    6 
#define US4_TRIG_PIN    17 
#define US4_ECHO_PIN    26 

// I2C Definitions
#define I2C_BUS             1     
#define TEMP_I2C_ADDRESS    0x76 
#define SOIL_I2C_ADDRESS    0x48 

// System Constants
#define DISTANCE_MIN    10.0 // cm
// --- End Pin Definitions ---


class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher()
    : Node("sensor_publisher"), count_(0)
    {
        RCLCPP_INFO(this->get_logger(), "SensorPublisher starting...");

        // NOTE: Since the motor_driver_node uses gpioInitialise() and runs with sudo,
        // we assume PiGPIO is initialized and running in the daemon/direct mode.
        // For the sensor publisher, we will try to connect as a client first, 
        // and fall back to direct access if needed, but given your working sensor output
        // we'll primarily use the pigpiod client interface (`gpioCfg*` functions).
        
        // --- 1. PiGPIO Initialization/Connection (Client Mode) ---
        // Setting up the client to communicate with the running pigpiod daemon
        int result = gpioCfgStart(0, 0); 
        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO client connection failed (Error: %d). Ensure 'pigpiod' daemon is running.", result);
            // Fallback: Try direct initialization (requires sudo/permissions)
            // This is unlikely to fix it if pigpiod is running, but good practice.
            if (gpioInitialise() < 0) {
                RCLCPP_ERROR(this->get_logger(), "PiGPIO direct initialization failed. The node may not function correctly without GPIO access.");
                return;
            }
        }
        RCLCPP_INFO(this->get_logger(), "PiGPIO client connected successfully.");

        setup_gpios(); // Initialize all pins and I2C connections

        // --- 2. ROS 2 Setup ---
        // Create a publisher for the custom sensor data message
        publisher_ = this->create_publisher<rpi_cpp_hardware::msg::RpiSensorData>("rpi_sensor_data", 10);
        
        // Create a timer to call the publish_sensor_data_loop function every 500ms
        timer_ = this->create_wall_timer(
            500ms, std::bind(&SensorPublisher::publish_sensor_data_loop, this));

        RCLCPP_INFO(this->get_logger(), "SensorPublisher ready, publishing to /rpi_sensor_data every 500ms...");
    }

    ~SensorPublisher() 
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SensorPublisher. Cleaning up PiGPIO connection.");
        cleanup_gpios();
    }

private:
    // --- Global Variables/Handles ---
    int temp_i2c_handle_ = -1;
    int soil_i2c_handle_ = -1;
    size_t count_;

    // --- ROS 2 Members ---
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<rpi_cpp_hardware::msg::RpiSensorData>::SharedPtr publisher_;

    /**
     * Reads the distance from an ultrasonic sensor. (Adapted from your script)
     * Note: Using gpioDelay (usleep equivalent) is common in pigpio direct mode, 
     * but less desirable in ROS 2 nodes. However, for precise timing, we keep it.
     */
    double read_ultrasonic(int trig_pin, int echo_pin) 
    {
        // 1. Trigger the sensor
        gpioWrite(trig_pin, PI_HIGH);
        gpioDelay(10); // 10 microsecond pulse
        gpioWrite(trig_pin, PI_LOW);

        long start_time = gpioTick();
        long end_time = start_time;
        long timeout = 50000; // 50ms timeout for max distance ~8m

        // 2. Wait for the echo start (HIGH)
        while (gpioRead(echo_pin) == PI_LOW && (gpioTick() - start_time) < timeout) {
            start_time = gpioTick();
        }

        // 3. Wait for the echo end (LOW)
        while (gpioRead(echo_pin) == PI_HIGH && (gpioTick() - start_time) < timeout) {
            end_time = gpioTick();
        }

        // Check for timeout or error
        if (gpioTick() - start_time >= timeout || end_time == start_time) {
            return -1.0; // Indicate error/timeout
        }

        long time_taken = end_time - start_time;
        // Speed of sound = 343 m/s = 0.0343 cm/us
        // Distance = (Time * Speed of Sound) / 2
        double distance = (double)time_taken * 0.0343 / 2.0;

        return distance;
    }

    /**
     * Placeholder for reading soil moisture using I2C ADC. (Adapted from your script)
     */
    int read_soil_moisture() 
    {
        if (soil_i2c_handle_ < 0) return -1; // Error

        // For now, return a mock value for demonstration.
        // In a real application, you'd use i2cReadByteData or similar.
        static int mock_moisture = 750;
        mock_moisture += (rand() % 10) - 5; // Jitter
        if (mock_moisture > 900) mock_moisture = 900;
        if (mock_moisture < 100) mock_moisture = 100;
        return mock_moisture;
    }

    /**
     * Placeholder for reading temperature from an I2C sensor. (Adapted from your script)
     */
    double read_temperature() 
    {
        if (temp_i2c_handle_ < 0) return -99.0; // Error

        // For now, return a mock value for demonstration.
        // In a real application, you'd use i2cRead* functions.
        static double mock_temp = 25.5;
        mock_temp += (rand() % 10 - 5) / 100.0; // Jitter +/- 0.05 C
        if (mock_temp > 30.0) mock_temp = 30.0;
        if (mock_temp < 20.0) mock_temp = 20.0;
        return mock_temp;
    }


    void setup_gpios() 
    {
        RCLCPP_INFO(this->get_logger(), "Setting up GPIOs and I2C connections...");

        // --- I2C Setup ---
        // Note: pigpiod_if is needed here for i2cOpen, as the client interface handles it.
        temp_i2c_handle_ = i2cOpen(I2C_BUS, TEMP_I2C_ADDRESS, 0);
        soil_i2c_handle_ = i2cOpen(I2C_BUS, SOIL_I2C_ADDRESS, 0);

        if (temp_i2c_handle_ < 0) RCLCPP_WARN(this->get_logger(), " I2C Temp Sensor failed to open.");
        if (soil_i2c_handle_ < 0) RCLCPP_WARN(this->get_logger(), " I2C Soil Sensor failed to open.");

        // --- Ultrasonic Sensor Setup ---
        // S1
        gpioSetMode(US1_TRIG_PIN, PI_OUTPUT);
        gpioSetMode(US1_ECHO_PIN, PI_INPUT);
        gpioWrite(US1_TRIG_PIN, 0); 
        
        // S4
        gpioSetMode(US4_TRIG_PIN, PI_OUTPUT);
        gpioSetMode(US4_ECHO_PIN, PI_INPUT);
        gpioWrite(US4_TRIG_PIN, 0);

        usleep(1000000); // Wait for pins to settle (1 second)
        RCLCPP_INFO(this->get_logger(), "GPIOs and I2C handles initialized.");
    }

    void cleanup_gpios() 
    {
        // Close I2C handles
        if (temp_i2c_handle_ >= 0) i2cClose(temp_i2c_handle_);
        if (soil_i2c_handle_ >= 0) i2cClose(soil_i2c_handle_);

        // Set used GPIOs back to input for safety
        gpioSetMode(US1_TRIG_PIN, PI_INPUT);
        gpioSetMode(US1_ECHO_PIN, PI_INPUT);
        gpioSetMode(US4_TRIG_PIN, PI_INPUT);
        gpioSetMode(US4_ECHO_PIN, PI_INPUT);

        // Terminate pigpio (only if initialized directly, but safe to call)
        // If connected as client, this terminates the client connection.
        gpioTerminate();
    }


    void publish_sensor_data_loop()
    {
        // --- 1. Read Sensors ---
        double us1_dist = read_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);
        double us4_dist = read_ultrasonic(US4_TRIG_PIN, US4_ECHO_PIN);
        
        int soil = read_soil_moisture();
        double temp = read_temperature();

        // --- 2. Create and Populate Message ---
        auto message = rpi_cpp_hardware::msg::RpiSensorData();
        message.header.stamp = this->now();
        message.header.frame_id = "rpi_base_link"; 

        message.us1_distance_cm = us1_dist;
        message.us4_distance_cm = us4_dist;
        message.soil_moisture_adc = soil;
        message.air_temperature_celsius = temp;

        // --- 3. Publish Message ---
        publisher_->publish(message);

        // --- 4. Log Status ---
        RCLCPP_INFO(
            this->get_logger(), 
            "Publishing Sensor Data [%zu]: US1=%.2f cm, US4=%.2f cm, Soil=%d ADC, Temp=%.2f C",
            count_++, 
            us1_dist, 
            us4_dist, 
            soil, 
            temp
        );
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorPublisher>());
    rclcpp::shutdown();
    return 0;
}
