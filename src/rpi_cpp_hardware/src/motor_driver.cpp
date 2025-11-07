// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <thread>   // Required for std::this_thread::sleep_for
// #include <chrono>   // Required for std::chrono::seconds
// #include <iomanip>
// #include <cmath>
// #include <pigpio.h>
// #include <pigpiod_if2.h>

// // Include the standard PiGPIO header for direct library access
// // extern "C" {
// //     #include <pigpio.h> 
// // }
// // extern "C" {
// //     #include <pigpiod_if.h> 
// // }

// // --- Pin Definitions (BCM Numbering) ---
// #define M1_PWM_PIN      18 
// #define M1_DIR_PIN      23
// #define M1_DIR2_PIN     27
// #define M2_PWM_PIN      12 
// #define M2_DIR_PIN      24
// #define M2_DIR2_PIN     22 

// // System Constants
// #define SPEED_MAX       255.0 // Max duty cycle for PWM (0-255)
// #define PWM_FREQUENCY   500   // PWM Frequency in Hz

// int pi_handle_ = -1;

// // NOTE: global_pigpio_handle has been removed.

// /**
//  * @brief Sets the speed of a motor by controlling the PWM duty cycle.
//  * NOTE: Using direct PiGPIO functions which DO NOT take a handle.
//  * @param pwm_pin The GPIO pin connected to the L298N ENA/ENB.
//  * @param speed The desired speed duty cycle (0.0 to 1.0, where 1.0 is max).
//  */
// void set_motor_speed(int pwm_pin, double speed) {
//     // Check pigpio initialization status
//     if (pi_handle_ < 0) return;

//     // Convert speed (0.0 to 1.0) to PWM duty cycle (0 to SPEED_MAX)
//     int duty_cycle = static_cast<int>(std::round(std::min(1.0, std::max(0.0, speed)) * SPEED_MAX));

//     if (pwm_pin == M1_PWM_PIN) {
//         // M1 uses hardware PWM. Duty cycle is 0 to 1,000,000.
//         // NOTE: No handle argument passed.
//         hardware_PWM(pi_handle_, pwm_pin, PWM_FREQUENCY, duty_cycle * 1000000 / SPEED_MAX); 
//     } else {
//         // M2 uses software PWM. Duty cycle is 0 to SPEED_MAX (255).
//         // NOTE: No handle argument passed.
//         set_PWM_dutycycle(pi_handle_, pwm_pin, duty_cycle);
//     }
// }

// /**
//  * @brief Sets the direction of a motor.
//  * NOTE: Using direct PiGPIO functions which DO NOT take a handle.
//  * @param dir_pin The primary direction pin (IN1 or IN3).
//  * @param forward true for one direction, false for reverse.
//  */
// void set_motor_direction(int dir_pin, bool forward) {
//     // Check pigpio initialization status
//     if (pi_handle_ < 0) return;
    
//     int dir2_pin = (dir_pin == M1_DIR_PIN) ? M1_DIR2_PIN : M2_DIR2_PIN;

//     if (forward) {
//         // Forward: Primary Pin (IN1/IN3) = HIGH, Companion Pin (IN2/IN4) = LOW
//         // NOTE: No handle argument passed.
//         gpio_write(pi_handle_, dir_pin, PI_HIGH);
//         gpio_write(pi_handle_, dir2_pin, PI_LOW);
//     } else {
//         // Reverse: Primary Pin (IN1/IN3) = LOW, Companion Pin (IN2/IN4) = HIGH
//         // NOTE: No handle argument passed.
//         gpio_write(pi_handle_, dir_pin, PI_LOW);
//         gpio_write(pi_handle_, dir2_pin, PI_HIGH);
//     }
// }

// class MotorDriver : public rclcpp::Node
// {
// public:
//     MotorDriver() : Node("motor_driver_node")
//     {
//         // 1. Initialize PiGPIO using direct access (gpioInitialise)
//         // This is necessary when not using the pigpio client interface.
//         // pigpio_handle_ = gpioInitialise();
//         pi_handle_ = pigpio_start(NULL, NULL);


//         if (pi_handle_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "PiGPIO direct initialization failed with error code: %d. Ensure the 'pigpiod' daemon is running or required libraries are available.", pigpio_handle_);
//         } else {
//             RCLCPP_INFO(this->get_logger(), "PiGPIO library initialized successfully (Version: %d).", pigpio_handle_);

//             // The one-second delay is less critical in direct mode but kept for robustness
//             RCLCPP_INFO(this->get_logger(), "Waiting 1 second for PiGPIO initialization...");
//             std::this_thread::sleep_for(std::chrono::seconds(1));

//             // 2. Setup Motor Pins (only if initialization succeeded)
//             setup_motor_gpios();
//         }

//         // 3. Create Subscriber
//         cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "/cmd_vel", 10, std::bind(&MotorDriver::cmd_vel_callback, this, std::placeholders::_1));
        
//         RCLCPP_INFO(this->get_logger(), "MotorDriver ready, listening to /cmd_vel...");
//     }

//     ~MotorDriver()
//     {
//         RCLCPP_INFO(this->get_logger(), "Shutting down MotorDriver. Stopping motors and cleaning up GPIO.");
//         if (pi_handle_ >= 0) {
//             set_motor_speed(M1_PWM_PIN, 0.0);
//             set_motor_speed(M2_PWM_PIN, 0.0);
//             // Terminate the local PiGPIO library
//             // gpioTerminate(); 
//             pigpio_stop(pi_handle_);
//         }
//     }

// private:
//     // This now stores the version number on success, or an error code on failure.
//     int pigpio_handle_;

//     void setup_motor_gpios()
//     {
//         // All direct PiGPIO functions called without a handle argument.

//         // M1 (Left) Setup
//         set_mode(pi_handle_, M1_PWM_PIN, PI_OUTPUT);
//         set_mode(pi_handle_, M1_DIR_PIN, PI_OUTPUT);
//         set_mode(pi_handle_, M1_DIR2_PIN, PI_OUTPUT);

//         // M2 (Right) Setup
//         set_mode(pi_handle_, M2_PWM_PIN, PI_OUTPUT);
//         set_mode(pi_handle_, M2_DIR_PIN, PI_OUTPUT);
//         set_mode(pi_handle_, M2_DIR2_PIN, PI_OUTPUT);
        
//         // M2 (Software PWM) range setup
//         // NOTE: No handle argument passed.
//         set_PWM_frequency(pi_handle_, M2_PWM_PIN, PWM_FREQUENCY);
//         set_PWM_range(pi_handle_, M2_PWM_PIN, SPEED_MAX);

//         // Initial stop
//         set_motor_speed(M1_PWM_PIN, 0.0);
//         set_motor_speed(M2_PWM_PIN, 0.0);
//     }

//     void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
//     {
//         // Only attempt motor control if PiGPIO initialization was successful
//         if (pi_handle_ < 0) return;

//         double linear_x = msg->linear.x;
//         double angular_z = msg->angular.z;

//         // Simplified Differential Drive Control Mapping:
//         double abs_speed = std::min(1.0, std::abs(linear_x));
//         double max_turn = 0.5;
        
//         double left_speed = abs_speed - (angular_z * max_turn);
//         double right_speed = abs_speed + (angular_z * max_turn);
        
//         bool forward = linear_x >= 0;
        
//         // Handle stopping
//         if (std::abs(linear_x) < 0.05 && std::abs(angular_z) < 0.05) {
//             set_motor_speed(M1_PWM_PIN, 0.0);
//             set_motor_speed(M2_PWM_PIN, 0.0);
//             RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot stopped.");
//             return;
//         }

//         // Apply direction (both motors follow the sign of linear_x)
//         set_motor_direction(M1_DIR_PIN, forward);
//         set_motor_direction(M2_DIR_PIN, forward);
        
//         // Apply differential speeds 
//         set_motor_speed(M1_PWM_PIN, std::abs(left_speed));
//         set_motor_speed(M2_PWM_PIN, std::abs(right_speed));

//         RCLCPP_INFO(this->get_logger(), "Received Command: Linear X=%.2f, Angular Z=%.2f | Left Speed=%.2f, Right Speed=%.2f", 
//             linear_x, angular_z, std::abs(left_speed), std::abs(right_speed));
//     }

//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     try {
//         auto node = std::make_shared<MotorDriver>();
//         rclcpp::spin(node);
//     } catch (const std::exception& e) {
//         RCLCPP_ERROR(rclcpp::get_logger("motor_driver_main"), "Exception caught: %s", e.what());
//     }
//     rclcpp::shutdown();
//     return 0;
// }


#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp> // Subscribing to String messages for simple commands
#include <thread>
#include <chrono>
#include <cmath>
#include <pigpio.h>
#include <pigpiod_if2.h>
#include <algorithm>

using namespace std::chrono_literals;

// --- Pin Definitions (BCM Numbering) ---
// Assuming a standard H-bridge setup (e.g., L298N) where two pins control direction 
// and one PWM pin controls speed (Enable).
#define M1_PWM_PIN      18  // PWM pin (often H-bridge Enable A)
#define M1_DIR_PIN      23  // Direction pin 1 (Input 1)
#define M1_DIR2_PIN     27  // Direction pin 2 (Input 2)
#define M2_PWM_PIN      12  // PWM pin (often H-bridge Enable B)
#define M2_DIR_PIN      24  // Direction pin 1 (Input 3)
#define M2_DIR2_PIN     22  // Direction pin 2 (Input 4)

// System Constants
#define SPEED_MAX       255.0 // Max duty cycle for PWM (pigpio range 0-255)
#define PWM_FREQUENCY   500   // PWM Frequency in Hz
#define DEFAULT_SPEED   150.0 // Default speed to move when commanded (0-255)

int pi_handle_ = -1; // PiGPIO daemon handle

// --- Helper Functions ---

/**
 * @brief Sets the speed of a motor by controlling the PWM duty cycle.
 * @param pwm_pin The GPIO pin connected to the PWM/Enable input.
 * @param duty_cycle The speed value (0.0 to 255.0).
 */
void set_motor_speed(int pwm_pin, double duty_cycle) {
    if (pi_handle_ < 0) return;
    
    // Clamp speed between 0 and 255
    duty_cycle = std::min(std::max(duty_cycle, 0.0), SPEED_MAX);
    
    // Use the pigpio function to set the hardware PWM duty cycle
    set_PWM_dutycycle(pi_handle_, pwm_pin, (unsigned int)duty_cycle);
}

/**
 * @brief Sets the direction of a motor by controlling the two direction pins.
 * @param dir_pin1 The first direction pin (e.g., Input 1).
 * @param dir_pin2 The second direction pin (e.g., Input 2).
 * @param forward True for forward motion, false for reverse motion.
 */
void set_motor_direction(int dir_pin1, int dir_pin2, bool forward) {
    if (pi_handle_ < 0) return;

    if (forward) {
        gpio_write(pi_handle_, dir_pin1, PI_HIGH);
        gpio_write(pi_handle_, dir_pin2, PI_LOW);
    } else {
        gpio_write(pi_handle_, dir_pin1, PI_LOW);
        gpio_write(pi_handle_, dir_pin2, PI_HIGH);
    }
}

class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver() : Node("motor_driver_node")
    {
        // 1. Initialize PiGPIO
        pi_handle_ = pigpio_start(NULL, NULL);

        if (pi_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO initialization failed.");
        } else {
            RCLCPP_INFO(this->get_logger(), "PiGPIO library initialized successfully.");
            setup_motor_gpios();
        }

        // 2. Create Subscriber for String Commands
        cmd_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "robot_command", 10, std::bind(&MotorDriver::command_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "MotorDriver ready, listening to /robot_command...");
    }

    ~MotorDriver()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MotorDriver. Stopping motors and cleaning up GPIO.");
        if (pi_handle_ >= 0) {
            // Stop motors on shutdown
            set_motor_speed(M1_PWM_PIN, 0.0);
            set_motor_speed(M2_PWM_PIN, 0.0);
            // Terminate pigpio client connection
            pigpio_stop(pi_handle_);
        }
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_subscription_;
    
    void setup_motor_gpios()
    {
        // Set all pins to OUTPUT mode
        set_mode(pi_handle_, M1_PWM_PIN, PI_OUTPUT);
        set_mode(pi_handle_, M1_DIR_PIN, PI_OUTPUT);
        set_mode(pi_handle_, M1_DIR2_PIN, PI_OUTPUT);
        set_mode(pi_handle_, M2_PWM_PIN, PI_OUTPUT);
        set_mode(pi_handle_, M2_DIR_PIN, PI_OUTPUT);
        set_mode(pi_handle_, M2_DIR2_PIN, PI_OUTPUT);
        
        // Start PWM for both motors
        set_PWM_frequency(pi_handle_, M1_PWM_PIN, PWM_FREQUENCY);
        set_PWM_frequency(pi_handle_, M2_PWM_PIN, PWM_FREQUENCY);
        
        // Initial state: STOPPED (0% duty cycle)
        set_motor_speed(M1_PWM_PIN, 0.0);
        set_motor_speed(M2_PWM_PIN, 0.0);
    }

    /**
     * @brief Interprets high-level string commands and sets the direction/speed.
     */
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (pi_handle_ < 0) return;

        const std::string& command = msg->data;
        
        if (command == "FORWARD") {
            set_motor_direction(M1_DIR_PIN, M1_DIR2_PIN, true);
            set_motor_direction(M2_DIR_PIN, M2_DIR2_PIN, true);
            set_motor_speed(M1_PWM_PIN, DEFAULT_SPEED);
            set_motor_speed(M2_PWM_PIN, DEFAULT_SPEED);
            RCLCPP_INFO(this->get_logger(), "Command: FORWARD (Speed: %.0f)", DEFAULT_SPEED);
            
        } else if (command == "REVERSE") {
            set_motor_direction(M1_DIR_PIN, M1_DIR2_PIN, false);
            set_motor_direction(M2_DIR_PIN, M2_DIR2_PIN, false);
            set_motor_speed(M1_PWM_PIN, DEFAULT_SPEED);
            set_motor_speed(M2_PWM_PIN, DEFAULT_SPEED);
            RCLCPP_INFO(this->get_logger(), "Command: REVERSE (Speed: %.0f)", DEFAULT_SPEED);
            
        } else if (command == "STOP" || command == "WATER") {
            set_motor_speed(M1_PWM_PIN, 0.0);
            set_motor_speed(M2_PWM_PIN, 0.0);
            RCLCPP_INFO(this->get_logger(), "Command: STOP/WATER. Motors stopped.");
            
        } else {
             RCLCPP_WARN(this->get_logger(), "Received unknown command: %s", command.c_str());
        }
        
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<MotorDriver>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_driver_main"), "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}