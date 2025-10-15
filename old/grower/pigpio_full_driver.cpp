#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pigpio.h>
#include <cmath>
#include <map>

using namespace std::chrono_literals;

// --- Motor Pin Definitions (BCM Numbering) ---
#define M1_PWM_PIN    18 // Left Motor PWM (HW PWM0)
#define M1_DIR_PIN    23 // Left Motor Direction A
#define M1_DIR2_PIN   27 // Left Motor Direction B
#define M2_PWM_PIN    12 // Right Motor PWM (SW PWM)
#define M2_DIR_PIN    24 // Right Motor Direction A
#define M2_DIR2_PIN   22 // Right Motor Direction B

// --- Ultrasonic Sensor Pin Definitions (BCM Numbering) ---
#define US1_TRIG_PIN  5  // Front
#define US1_ECHO_PIN  6 
#define US2_TRIG_PIN  16 // Left
#define US2_ECHO_PIN  20 
#define US3_TRIG_PIN  21 // Right
#define US3_ECHO_PIN  25 // (Conflict resolved, using BCM 25)
#define US4_TRIG_PIN  17 // Rear
#define US4_ECHO_PIN  26 

// --- I2C & Constants ---
#define I2C_BUS         1
#define TEMP_I2C_ADDRESS    0x76
#define SOIL_I2C_ADDRESS    0x48
#define SPEED_MAX     255
#define PWM_FREQUENCY 500

class PigpioFullDriver : public rclcpp::Node
{
public:
    PigpioFullDriver() : Node("pigpio_full_driver")
    {
        // 1. Initialize pigpio
        if (gpioInitialise() < 0) {
            RCLCPP_ERROR(this->get_logger(), "PiGPIO failed to initialize!");
            throw std::runtime_error("PiGPIO Init Failure");
        }
        RCLCPP_INFO(this->get_logger(), "PiGPIO initialized. Setting up pins & I2C...");
        setup_gpios();

        // 2. Publishers for all sensors
        pub_us1_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasound/front", 10);
        pub_us2_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasound/left", 10);
        pub_us3_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasound/right", 10);
        pub_us4_ = this->create_publisher<sensor_msgs::msg::Range>("ultrasound/rear", 10);
        pub_soil_ = this->create_publisher<std_msgs::msg::Int32>("sensor/soil_moisture", 10);
        pub_temp_ = this->create_publisher<std_msgs::msg::Float64>("sensor/air_temp", 10);
        
        // 3. Subscriber for motor commands
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&PigpioFullDriver::cmd_vel_callback, this, std::placeholders::_1));

        // 4. Timer for periodic sensor reading (200ms interval for all)
        sensor_timer_ = this->create_wall_timer(
            200ms, std::bind(&PigpioFullDriver::sensor_timer_callback, this));
    }

    ~PigpioFullDriver()
    {
        // Stop motors and cleanup
        set_motor_speed(M1_PWM_PIN, 0);
        set_motor_speed(M2_PWM_PIN, 0);
        if (temp_i2c_handle >= 0) i2cClose(temp_i2c_handle);
        if (soil_i2c_handle >= 0) i2cClose(soil_i2c_handle);
        gpioTerminate();
        RCLCPP_INFO(this->get_logger(), "PiGPIO terminated.");
    }

private:
    // I2C Handles
    int temp_i2c_handle = -1;
    int soil_i2c_handle = -1;

    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub_us1_, pub_us2_, pub_us3_, pub_us4_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_soil_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_temp_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;

    void setup_gpios()
    {
        // I2C Setup (Mocks successful opening if not actually running on Pi)
        temp_i2c_handle = i2cOpen(I2C_BUS, TEMP_I2C_ADDRESS, 0);
        soil_i2c_handle = i2cOpen(I2C_BUS, SOIL_I2C_ADDRESS, 0);

        // Motor Setup (M1 & M2)
        std::vector<int> motor_pins = {M1_PWM_PIN, M1_DIR_PIN, M1_DIR2_PIN, M2_PWM_PIN, M2_DIR_PIN, M2_DIR2_PIN};
        for (int pin : motor_pins) {
            gpioSetMode(pin, PI_OUTPUT);
        }
        gpioSetPWMfrequency(M2_PWM_PIN, PWM_FREQUENCY);
        gpioSetPWMrange(M2_PWM_PIN, SPEED_MAX);
        set_motor_speed(M1_PWM_PIN, 0);
        set_motor_speed(M2_PWM_PIN, 0);

        // Ultrasonic Sensor Setup
        std::map<int, int> us_pins = {
            {US1_TRIG_PIN, US1_ECHO_PIN}, {US2_TRIG_PIN, US2_ECHO_PIN}, 
            {US3_TRIG_PIN, US3_ECHO_PIN}, {US4_TRIG_PIN, US4_ECHO_PIN}
        };
        for (auto const& [trig, echo] : us_pins) {
            gpioSetMode(trig, PI_OUTPUT);
            gpioSetMode(echo, PI_INPUT);
            gpioWrite(trig, 0);
        }
        sleep(1);
    }

    // --- PIGPIO CORE FUNCTIONS ---

    void set_motor_speed(int pwm_pin, int speed) {
        if (speed < 0) speed = 0;
        if (speed > SPEED_MAX) speed = SPEED_MAX;
        
        if (pwm_pin == M1_PWM_PIN) {
            // Hardware PWM
            gpioHardwarePWM(M1_PWM_PIN, PWM_FREQUENCY, speed * 1000000 / SPEED_MAX); 
        } else {
            // Software PWM
            gpioPWM(M2_PWM_PIN, speed);
        }
    }

    void set_motor_direction(int dir_pin, int dir2_pin, bool forward) {
        if (forward) {
            gpioWrite(dir_pin, PI_HIGH);
            gpioWrite(dir2_pin, PI_LOW);
        } else {
            gpioWrite(dir_pin, PI_LOW);
            gpioWrite(dir2_pin, PI_HIGH);
        }
    }

    double read_ultrasonic(int trig_pin, int echo_pin) {
        // (Robust ultrasonic code with delays to prevent CPU lockup)
        gpioWrite(trig_pin, PI_HIGH);
        gpioDelay(10);
        gpioWrite(trig_pin, PI_LOW);

        long start_time = gpioTick();
        long end_time = start_time;
        long timeout = 50000;

        while (gpioRead(echo_pin) == PI_LOW && (gpioTick() - start_time) < timeout) { start_time = gpioTick(); gpioDelay(1); }
        while (gpioRead(echo_pin) == PI_HIGH && (gpioTick() - start_time) < timeout) { end_time = gpioTick(); gpioDelay(1); }

        if (gpioTick() - start_time >= timeout || end_time == start_time) {
            return 999.0; // Error/Timeout in cm
        }

        long time_taken = end_time - start_time;
        return (double)time_taken * 0.0343 / 2.0; // Distance in cm
    }

    int read_soil_moisture() {
        if (soil_i2c_handle < 0) return -1;
        // Mock I2C read
        static int mock_moisture = 750;
        mock_moisture += (rand() % 10) - 5;
        return mock_moisture;
    }

    double read_temperature() {
        if (temp_i2c_handle < 0) return -99.0;
        // Mock I2C read
        static double mock_temp = 25.5;
        mock_temp += (rand() % 20 - 10) / 10.0; 
        return mock_temp;
    }

    // --- ROS CALLBACKS ---

    // Reads all sensors and publishes their data
    void sensor_timer_callback()
    {
        // 1. Ultrasonic Sensor Reads
        // US1 (Front)
        double dist1 = read_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);
        publish_range(pub_us1_, "us1_link", dist1);

        // US2 (Left)
        double dist2 = read_ultrasonic(US2_TRIG_PIN, US2_ECHO_PIN);
        publish_range(pub_us2_, "us2_link", dist2);
        
        // US3 (Right)
        double dist3 = read_ultrasonic(US3_TRIG_PIN, US3_ECHO_PIN);
        publish_range(pub_us3_, "us3_link", dist3);

        // US4 (Rear)
        double dist4 = read_ultrasonic(US4_TRIG_PIN, US4_ECHO_PIN);
        publish_range(pub_us4_, "us4_link", dist4);

        // 2. I2C Sensor Reads
        auto soil_msg = std_msgs::msg::Int32();
        soil_msg.data = read_soil_moisture();
        pub_soil_->publish(soil_msg);

        auto temp_msg = std_msgs::msg::Float64();
        temp_msg.data = read_temperature();
        pub_temp_->publish(temp_msg);
    }

    // Helper to publish standardized range messages
    void publish_range(rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr pub, const std::string& frame_id, double distance_cm) {
        auto range_msg = sensor_msgs::msg::Range();
        range_msg.header.stamp = this->now();
        range_msg.header.frame_id = frame_id;
        range_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
        range_msg.field_of_view = 0.523; // 30 degrees (approx)
        range_msg.min_range = 0.02; 
        range_msg.max_range = 4.00; 
        range_msg.range = distance_cm / 100.0; // Convert cm to meters (ROS standard)
        pub->publish(range_msg);
    }


    // Processes Twist messages and translates them into differential drive commands
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Twist message contains linear.x (forward/backward speed) and angular.z (turning speed)
        double linear = msg->linear.x; 
        double angular = msg->angular.z;
        
        // --- Differential Drive Model (Simplified) ---
        double left_power = linear - (angular * 0.5); // Turn left, slow down left motor
        double right_power = linear + (angular * 0.5); // Turn left, speed up right motor

        // Scale power from [-1.0, 1.0] to duty cycle [0, SPEED_MAX]
        int speed_left = static_cast<int>(std::abs(left_power) * SPEED_MAX);
        int speed_right = static_cast<int>(std::abs(right_power) * SPEED_MAX);

        // Set directions
        set_motor_direction(M1_DIR_PIN, M1_DIR2_PIN, left_power >= 0); // M1 is Left
        set_motor_direction(M2_DIR_PIN, M2_DIR2_PIN, right_power >= 0); // M2 is Right

        // Set speeds
        set_motor_speed(M1_PWM_PIN, speed_left);
        set_motor_speed(M2_PWM_PIN, speed_right);

        RCLCPP_INFO(this->get_logger(), "Rx Vel: L=%.2f, A=%.2f | M1: %s %d, M2: %s %d", 
                    linear, angular, 
                    left_power >= 0 ? "Fwd" : "Bwd", speed_left, 
                    right_power >= 0 ? "Fwd" : "Bwd", speed_right);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<PigpioFullDriver>());
    } catch (const std::runtime_error& e) {
        std::cerr << "Driver Node Halted: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}