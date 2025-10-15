#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pigpio.h>
#include <iostream>
#include <cmath>
#include <random> // For mock data when I2C fails

// --- Pin Definitions (BCM Numbering) ---
// Ultrasonic Sensor Pins (HC-SR04)
#define US1_TRIG_PIN  5
#define US1_ECHO_PIN  6
#define US4_TRIG_PIN  17 
#define US4_ECHO_PIN  26 

// I2C Definitions
#define I2C_BUS             1
#define TEMP_I2C_ADDRESS    0x76 
#define SOIL_I2C_ADDRESS    0x48 

// System Constants
#define ULTRASONIC_TIMEOUT_US 50000 // 50ms timeout

// Global Handles
int temp_i2c_handle = -1;
int soil_i2c_handle = -1;

/**
 * @brief Reads the distance from an ultrasonic sensor using pulse duration.
 * @param trig_pin The trigger GPIO pin.
 * @param echo_pin The echo GPIO pin.
 * @return Distance in centimeters, or a very high value (999.0) on timeout/error.
 */
double read_ultrasonic(int trig_pin, int echo_pin) {
    // 1. Trigger the sensor
    gpioWrite(trig_pin, PI_HIGH);
    gpioDelay(10); // 10 microsecond pulse
    gpioWrite(trig_pin, PI_LOW);

    long start_tick = gpioTick();
    long end_tick = start_tick;

    // 2. Wait for the echo start (HIGH)
    while (gpioRead(echo_pin) == PI_LOW && (gpioTick() - start_tick) < ULTRASONIC_TIMEOUT_US) {
        start_tick = gpioTick();
    }

    // 3. Wait for the echo end (LOW)
    while (gpioRead(echo_pin) == PI_HIGH && (gpioTick() - start_tick) < ULTRASONIC_TIMEOUT_US) {
        end_tick = gpioTick();
    }

    // Check for timeout or zero duration
    if (gpioTick() - start_tick >= ULTRASONIC_TIMEOUT_US || end_tick <= start_tick) {
        return 999.0; 
    }

    long time_taken_us = end_tick - start_tick;
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (Time * Speed of Sound) / 2
    double distance_cm = (double)time_taken_us * 0.0343 / 2.0;

    return distance_cm;
}

/**
 * @brief Reads soil moisture (I2C placeholder).
 * @return Mock or actual ADC reading.
 */
int read_soil_moisture() {
    if (soil_i2c_handle < 0) {
        // Return mock value if I2C failed to open
        static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        static std::uniform_int_distribution<int> distribution(100, 900);
        return distribution(generator);
    }
    // TODO: Implement actual I2C read command for 0x48 ADC chip
    return 750; // Mock value
}

/**
 * @brief Reads air temperature (I2C placeholder).
 * @return Mock or actual temperature in Celsius.
 */
double read_temperature() {
    if (temp_i2c_handle < 0) {
        // Return mock value if I2C failed to open
        static std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
        static std::uniform_real_distribution<double> distribution(20.0, 30.0);
        return distribution(generator);
    }
    // TODO: Implement actual I2C read command for 0x76 Temp sensor
    return 25.5; // Mock value
}


class SensorPublisher : public rclcpp::Node
{
public:
    SensorPublisher() : Node("sensor_publisher_node")
    {
        // 1. Initialize PiGPIO
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO initialization failed. Check daemon status.");
            throw std::runtime_error("PiGPIO initialization failed.");
        }
        RCLCPP_INFO(this->get_logger(), "PiGPIO initialized successfully for sensor control.");

        // 2. Setup Pins and I2C
        setup_sensor_gpios_and_i2c();

        // 3. Create Publishers
        pub_us1_ = this->create_publisher<std_msgs::msg::Float64>("distance_us1", 10);
        pub_us4_ = this->create_publisher<std_msgs::msg::Float64>("distance_us4", 10);
        pub_temp_ = this->create_publisher<std_msgs::msg::Float64>("air_temp", 10);
        pub_soil_ = this->create_publisher<std_msgs::msg::Int32>("soil_moisture", 10);

        // 4. Create Timer (Publish rate: 5 Hz or every 200ms)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&SensorPublisher::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "SensorPublisher ready, publishing sensor data every 200ms.");
    }

    ~SensorPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down SensorPublisher. Cleaning up GPIO and I2C.");
        if (temp_i2c_handle >= 0) i2cClose(temp_i2c_handle);
        if (soil_i2c_handle >= 0) i2cClose(soil_i2c_handle);
        gpioTerminate();
    }

private:
    void setup_sensor_gpios_and_i2c()
    {
        // I2C Setup
        temp_i2c_handle = i2cOpen(I2C_BUS, TEMP_I2C_ADDRESS, 0);
        soil_i2c_handle = i2cOpen(I2C_BUS, SOIL_I2C_ADDRESS, 0);

        if (temp_i2c_handle < 0) RCLCPP_WARN(this->get_logger(), "I2C Temp Sensor failed to open. Using mock data.");
        if (soil_i2c_handle < 0) RCLCPP_WARN(this->get_logger(), "I2C Soil Sensor failed to open. Using mock data.");

        // Ultrasonic Sensor Setup (Only US1 and US4 implemented for brevity)
        gpioSetMode(US1_TRIG_PIN, PI_OUTPUT);
        gpioSetMode(US1_ECHO_PIN, PI_INPUT);
        gpioWrite(US1_TRIG_PIN, 0);

        gpioSetMode(US4_TRIG_PIN, PI_OUTPUT);
        gpioSetMode(US4_ECHO_PIN, PI_INPUT);
        gpioWrite(US4_TRIG_PIN, 0);
        
        gpioDelay(1000000); // Wait 1 second for pins to settle
    }

    void timer_callback()
    {
        auto us1_msg = std_msgs::msg::Float64();
        auto us4_msg = std_msgs::msg::Float64();
        auto temp_msg = std_msgs::msg::Float64();
        auto soil_msg = std_msgs::msg::Int32();

        // 1. Read Ultrasonic Sensors (Distance in cm)
        us1_msg.data = read_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);
        us4_msg.data = read_ultrasonic(US4_TRIG_PIN, US4_ECHO_PIN);

        // 2. Read I2C Sensors
        temp_msg.data = read_temperature();
        soil_msg.data = read_soil_moisture();

        // 3. Publish
        pub_us1_->publish(us1_msg);
        pub_us4_->publish(us4_msg);
        pub_temp_->publish(temp_msg);
        pub_soil_->publish(soil_msg);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
            "PUB | US1: %.1f cm | US4: %.1f cm | Temp: %.1f C | Soil: %d", 
            us1_msg.data, us4_msg.data, temp_msg.data, soil_msg.data);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_us1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_us4_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_temp_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_soil_;
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
