#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

// Using pigpiod_if.h for the client interface, which resolves functions 
// to gpiod... or uses the handle as the first argument in many cases.
#include <pigpiod_if.h> 

#include <cmath>

// --- Pin Definitions (BCM Numbering) ---
#define M1_PWM_PIN    18 
#define M1_DIR_PIN    23
#define M1_DIR2_PIN   27
#define M2_PWM_PIN    12 
#define M2_DIR_PIN    24
#define M2_DIR2_PIN   22 

// System Constants
#define SPEED_MAX       255.0 // Max duty cycle for PWM (0-255)
#define PWM_FREQUENCY   500   // PWM Frequency in Hz

// --- Global Variable for Connection Handle ---
int global_pigpio_handle = -1; 

/**
 * @brief Sets the speed of a motor by controlling the PWM duty cycle.
 * NOTE: Using gpiod* functions, which explicitly require the handle.
 * @param pwm_pin The GPIO pin connected to the L298N ENA/ENB.
 * @param speed The desired speed duty cycle (0.0 to 1.0, where 1.0 is max).
 */
void set_motor_speed(int pwm_pin, double speed) {
    if (global_pigpio_handle < 0) return;

    // Convert speed (0.0 to 1.0) to PWM duty cycle (0 to SPEED_MAX)
    int duty_cycle = static_cast<int>(std::round(std::min(1.0, std::max(0.0, speed)) * SPEED_MAX));

    if (pwm_pin == M1_PWM_PIN) {
        // gpiodHardwarePWM(handle, gpio, PWMfreq, PWMduty) - Takes 4 arguments
        gpiodHardwarePWM(global_pigpio_handle, pwm_pin, PWM_FREQUENCY, duty_cycle * 1000000 / SPEED_MAX); 
    } else {
        // gpiodPWM(handle, user_gpio, dutycycle) - Takes 3 arguments
        gpiodPWM(global_pigpio_handle, pwm_pin, duty_cycle);
    }
}

/**
 * @brief Sets the direction of a motor.
 * NOTE: Using gpiodWrite function which explicitly takes the handle.
 * @param dir_pin The primary direction pin (IN1 or IN3).
 * @param forward true for one direction, false for reverse.
 */
void set_motor_direction(int dir_pin, bool forward) {
    if (global_pigpio_handle < 0) return;
    
    int dir2_pin = (dir_pin == M1_DIR_PIN) ? M1_DIR2_PIN : M2_DIR2_PIN;

    if (forward) {
        // Forward: Primary Pin (IN1/IN3) = HIGH, Companion Pin (IN2/IN4) = LOW
        // gpiodWrite(handle, gpio, level) - Takes 3 arguments
        gpiodWrite(global_pigpio_handle, dir_pin, PI_HIGH);
        gpiodWrite(global_pigpio_handle, dir2_pin, PI_LOW);
    } else {
        // Reverse: Primary Pin (IN1/IN3) = LOW, Companion Pin (IN2/IN4) = HIGH
        gpiodWrite(global_pigpio_handle, dir_pin, PI_LOW);
        gpiodWrite(global_pigpio_handle, dir2_pin, PI_HIGH);
    }
}

class MotorDriver : public rclcpp::Node
{
public:
    MotorDriver() : Node("motor_driver_node")
    {
        // 1. Initialize PiGPIO (Connect as a client to the running daemon)
        // gpioConnect is the correct client function name to connect to the daemon.
        pigpio_handle_ = gpioConnect(NULL, NULL);
        if (pigpio_handle_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO connection failed with error code: %d. Ensure the 'pigpiod' daemon is running.", pigpio_handle_);
            // Do not throw an error here, the application can continue (though without GPIO functionality)
        } else {
            // Set the global handle 
            global_pigpio_handle = pigpio_handle_;
            RCLCPP_INFO(this->get_logger(), "PiGPIO client connected successfully (Handle: %d).", pigpio_handle_);

            // 2. Setup Motor Pins (only if connection succeeded)
            setup_motor_gpios();
        }

        // 3. Create Subscriber
        cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&MotorDriver::cmd_vel_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "MotorDriver ready, listening to /cmd_vel...");
    }

    ~MotorDriver()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down MotorDriver. Stopping motors and cleaning up GPIO.");
        if (global_pigpio_handle >= 0) {
            set_motor_speed(M1_PWM_PIN, 0.0);
            set_motor_speed(M2_PWM_PIN, 0.0);
            // gpioTerminate(handle) is the client termination function.
            gpioTerminate(pigpio_handle_); 
        }
    }

private:
    int pigpio_handle_;

    void setup_motor_gpios()
    {
        // Using gpiodSetMode, gpiodSetPWMfrequency, gpiodSetPWMrange which explicitly take the handle.

        // M1 (Left) Setup
        gpiodSetMode(pigpio_handle_, M1_PWM_PIN, PI_OUTPUT);
        gpiodSetMode(pigpio_handle_, M1_DIR_PIN, PI_OUTPUT);
        gpiodSetMode(pigpio_handle_, M1_DIR2_PIN, PI_OUTPUT);

        // M2 (Right) Setup
        gpiodSetMode(pigpio_handle_, M2_PWM_PIN, PI_OUTPUT);
        gpiodSetMode(pigpio_handle_, M2_DIR_PIN, PI_OUTPUT);
        gpiodSetMode(pigpio_handle_, M2_DIR2_PIN, PI_OUTPUT);
        
        // M2 (Software PWM) range setup
        gpiodSetPWMfrequency(pigpio_handle_, M2_PWM_PIN, PWM_FREQUENCY);
        gpiodSetPWMrange(pigpio_handle_, M2_PWM_PIN, SPEED_MAX);

        // Initial stop
        set_motor_speed(M1_PWM_PIN, 0.0);
        set_motor_speed(M2_PWM_PIN, 0.0);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Only attempt motor control if PiGPIO connection was successful
        if (global_pigpio_handle < 0) return;

        double linear_x = msg->linear.x;
        double angular_z = msg->angular.z;

        // Simplified Differential Drive Control Mapping:
        double abs_speed = std::min(1.0, std::abs(linear_x));
        double max_turn = 0.5;
        
        double left_speed = abs_speed - (angular_z * max_turn);
        double right_speed = abs_speed + (angular_z * max_turn);
        
        bool forward = linear_x >= 0;
        
        // Handle stopping
        if (std::abs(linear_x) < 0.05 && std::abs(angular_z) < 0.05) {
            set_motor_speed(M1_PWM_PIN, 0.0);
            set_motor_speed(M2_PWM_PIN, 0.0);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Robot stopped.");
            return;
        }

        // Apply direction (both motors follow the sign of linear_x)
        set_motor_direction(M1_DIR_PIN, forward);
        set_motor_direction(M2_DIR_PIN, forward);
        
        // Apply differential speeds 
        set_motor_speed(M1_PWM_PIN, std::abs(left_speed));
        set_motor_speed(M2_PWM_PIN, std::abs(right_speed));

        RCLCPP_INFO(this->get_logger(), "Received Command: Linear X=%.2f, Angular Z=%.2f | Left Speed=%.2f, Right Speed=%.2f", 
            linear_x, angular_z, std::abs(left_speed), std::abs(right_speed));
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
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
