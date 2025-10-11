#include <iostream>
#include <unistd.h>
#include <pigpio.h>
#include <iomanip> // For setprecision
#include <cmath>   // For M_PI, etc. if needed

// --- Global Variables ---
// Handles for I2C communication
int temp_i2c_handle = -1;
int soil_i2c_handle = -1;

// --- Pin Definitions (BCM Numbering) ---

// Motor Pins (L298N)
#define M1_PWM_PIN    18 // Pin 12 - Hardware PWM0 for speed (L298N ENA)
#define M1_DIR_PIN    23 // Pin 16 - Digital output for direction (L298N IN1)
// Note: L298N IN2 must be wired permanently to GND for this single-pin direction logic.
#define M2_PWM_PIN    12 // Pin 32 - Software PWM for speed (L298N ENB)
#define M2_DIR_PIN    24 // Pin 18 - Digital output for direction (L298N IN3)
// Note: L298N IN4 must be wired permanently to GND for this single-pin direction logic.

// Ultrasonic Sensor Pins (HC-SR04)
#define US1_TRIG_PIN  5  // Pin 29 - Output
#define US1_ECHO_PIN  6  // Pin 31 - Input
#define US2_TRIG_PIN  16 // Pin 36 - Output
#define US2_ECHO_PIN  20 // Pin 38 - Input
#define US3_TRIG_PIN  21 // Pin 40 - Output
#define US3_ECHO_PIN  26 // Pin 37 - Input

// I2C Definitions
#define I2C_BUS         1    // Raspberry Pi's standard I2C bus (Pins 3 & 5)
#define TEMP_I2C_ADDRESS    0x76 // Example I2C address for Temperature Sensor
#define SOIL_I2C_ADDRESS    0x48 // Example I2C address for a common ADC chip

// System Constants
#define SPEED_MAX       255 // Max duty cycle for PWM
#define SPEED_FAST      200 // Defined SPEED_FAST for obstacle avoidance
#define SPEED_MODERATE  150 // Standard running speed
#define DISTANCE_MIN    20.0 // cm
#define SOIL_WET_LEVEL  500 // Example analog reading threshold

// --- Function Prototypes ---
void setup_gpios();
void cleanup_gpios();
void set_motor_speed(int pwm_pin, int speed);
void set_motor_direction(int dir_pin, bool forward);
double read_ultrasonic(int trig_pin, int echo_pin);
void handle_obstacle_avoidance();
int read_soil_moisture();
double read_temperature();

// --- Main Program ---
int main(int argc, char *argv[]) {
    std::cout << "Starting PiGPIO Complex Controller (2 Motors)..." << std::endl;

    if (gpioInitialise() < 0) {
        std::cerr << "PiGPIO initialization failed. Ensure pigpiod is running and you have privileges (sudo)." << std::endl;
        return 1;
    }

    std::cout << "PiGPIO initialized successfully." << std::endl;
    setup_gpios(); // Initialize all pins and I2C connections

    // --- Motor Direction Demonstration Sequence (M1) ---
    std::cout << "\nStarting Motor 1 Direction Test (Forward then Reverse)..." << std::endl;

    // 1. Run Motor 1 FORWARD
    std::cout << " [M1] Running FORWARD..." << std::endl;
    set_motor_direction(M1_DIR_PIN, true); // true = Forward
    set_motor_speed(M1_PWM_PIN, SPEED_MODERATE);
    sleep(2); // Run for 2 seconds

    // 2. Stop Motor 1 momentarily
    std::cout << " [M1] Stopping..." << std::endl;
    set_motor_speed(M1_PWM_PIN, 0);
    sleep(1); // Wait 1 second

    // 3. Run Motor 1 BACKWARD
    std::cout << " [M1] Running BACKWARD..." << std::endl;
    set_motor_direction(M1_DIR_PIN, false); // false = Reverse
    set_motor_speed(M1_PWM_PIN, SPEED_MODERATE);
    sleep(2); // Run for 2 seconds

    // 4. Final Stop
    std::cout << " [M1] Stopping." << std::endl;
    set_motor_speed(M1_PWM_PIN, 0);
    sleep(1);

    // --- Main Control Loop (Placeholder for real robot logic) ---
    std::cout << "\nStarting main control loop (Placeholder). Press Ctrl+C to exit." << std::endl;
    while (true) {
        // --- 1. Read Sensors ---
        double distance_front = read_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);
        int soil_moisture = read_soil_moisture();
        double air_temp = read_temperature();

        // --- 2. Print Status ---
        std::cout << "\n--- Status Report ---" << std::endl;
        std::cout << std::fixed << std::setprecision(2);
        std::cout << " | Front Distance: " << distance_front << " cm" << std::endl;
        std::cout << " | Soil Moisture: " << soil_moisture << " (Threshold: " << SOIL_WET_LEVEL << ")" << std::endl;
        std::cout << " | Air Temperature: " << air_temp << " C" << std::endl;

        // --- 3. Decision Logic ---
        handle_obstacle_avoidance();

        if (soil_moisture < SOIL_WET_LEVEL) {
            std::cout << " | ACTION: Soil is dry. Watering needed." << std::endl;
        } else {
            std::cout << " | STATUS: Soil moisture OK." << std::endl;
        }

        sleep(3); // Check sensors every 3 seconds
    }

    // --- Cleanup (Unreachable in infinite loop but good practice) ---
    std::cout << "Control sequence complete. Cleaning up..." << std::endl;
    cleanup_gpios(); // Clean up all pins and terminate pigpio
    return 0;
}

// --- Function Implementations ---

void setup_gpios() {
    std::cout << "Setting up GPIOs and I2C connections..." << std::endl;

    // --- I2C Setup ---
    temp_i2c_handle = i2cOpen(I2C_BUS, TEMP_I2C_ADDRESS, 0);
    soil_i2c_handle = i2cOpen(I2C_BUS, SOIL_I2C_ADDRESS, 0);

    if (temp_i2c_handle < 0) std::cerr << " I2C Temp Sensor failed to open." << std::endl;
    if (soil_i2c_handle < 0) std::cerr << " I2C Soil Sensor failed to open." << std::endl;

    // --- Motor Pin Setup ---
    // M1 (HW PWM)
    gpioSetMode(M1_PWM_PIN, PI_OUTPUT);
    gpioSetMode(M1_DIR_PIN, PI_OUTPUT);

    // M2 (SW PWM)
    gpioSetMode(M2_PWM_PIN, PI_OUTPUT);
    gpioSetMode(M2_DIR_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(M2_PWM_PIN, 500); // Set software PWM frequency
    gpioSetPWMrange(M2_PWM_PIN, SPEED_MAX);

    // Initial stop for all motors
    set_motor_speed(M1_PWM_PIN, 0);
    set_motor_speed(M2_PWM_PIN, 0);

    // --- Ultrasonic Sensor Setup ---
    // S1
    gpioSetMode(US1_TRIG_PIN, PI_OUTPUT);
    gpioSetMode(US1_ECHO_PIN, PI_INPUT);
    gpioWrite(US1_TRIG_PIN, 0); // Ensure trigger pin is low initially
    // S2
    gpioSetMode(US2_TRIG_PIN, PI_OUTPUT);
    gpioSetMode(US2_ECHO_PIN, PI_INPUT);
    gpioWrite(US2_TRIG_PIN, 0);
    // S3
    gpioSetMode(US3_TRIG_PIN, PI_OUTPUT);
    gpioSetMode(US3_ECHO_PIN, PI_INPUT);
    gpioWrite(US3_TRIG_PIN, 0);

    sleep(1); // Wait for pins to settle
}

void cleanup_gpios() {
    // Stop all motors
    set_motor_speed(M1_PWM_PIN, 0);
    set_motor_speed(M2_PWM_PIN, 0);

    // Close I2C handles
    if (temp_i2c_handle >= 0) i2cClose(temp_i2c_handle);
    if (soil_i2c_handle >= 0) i2cClose(soil_i2c_handle);

    // Set all used GPIOs back to input for safety
    gpioSetMode(M1_PWM_PIN, PI_INPUT);
    gpioSetMode(M1_DIR_PIN, PI_INPUT);
    gpioSetMode(M2_PWM_PIN, PI_INPUT);
    gpioSetMode(M2_DIR_PIN, PI_INPUT);
    gpioSetMode(US1_TRIG_PIN, PI_INPUT);
    gpioSetMode(US1_ECHO_PIN, PI_INPUT);
    gpioSetMode(US2_TRIG_PIN, PI_INPUT);
    gpioSetMode(US2_ECHO_PIN, PI_INPUT);
    gpioSetMode(US3_TRIG_PIN, PI_INPUT);
    gpioSetMode(US3_ECHO_PIN, PI_INPUT);

    // Terminate pigpio
    gpioTerminate();
    std::cout << "PiGPIO terminated." << std::endl;
}

/**
 * Sets the speed of a motor by controlling the PWM duty cycle.
 * @param pwm_pin The GPIO pin connected to the L298N ENA/ENB.
 * @param speed The duty cycle (0-255).
 */
void set_motor_speed(int pwm_pin, int speed) {
    if (speed < 0 || speed > SPEED_MAX) {
        speed = (speed < 0) ? 0 : SPEED_MAX;
    }
    // M1_PWM_PIN (18) uses Hardware PWM, M2_PWM_PIN (12) uses Software PWM
    if (pwm_pin == M1_PWM_PIN) {
        gpioHardwarePWM(pwm_pin, 500, speed * 1000000 / SPEED_MAX); // 500 Hz, duty cycle in microseconds (0-1,000,000)
    } else {
        gpioPWM(pwm_pin, speed); // Set software PWM duty cycle (0-255)
    }
}

/**
 * Sets the direction of a motor.
 * Assuming IN2/IN4 is wired to GND, true/HIGH sets one direction, false/LOW sets the reverse.
 * @param dir_pin The GPIO pin connected to the L298N IN1/IN3.
 * @param forward true for one direction (HIGH), false for the reverse (LOW).
 */
void set_motor_direction(int dir_pin, bool forward) {
    gpioWrite(dir_pin, forward ? 1 : 0);
}

/**
 * Reads the distance from an ultrasonic sensor.
 * @param trig_pin The trigger GPIO pin.
 * @param echo_pin The echo GPIO pin.
 * @return Distance in centimeters, or a very high value on timeout/error.
 */
double read_ultrasonic(int trig_pin, int echo_pin) {
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

    // Check for timeout
    if (gpioTick() - start_time >= timeout || end_time == start_time) {
        return 999.0; // Indicate error/timeout (max distance)
    }

    long time_taken = end_time - start_time;
    // Speed of sound = 343 m/s = 0.0343 cm/us
    // Distance = (Time * Speed of Sound) / 2
    double distance = (double)time_taken * 0.0343 / 2.0;

    return distance;
}

/**
 * Placeholder function for obstacle avoidance logic.
 */
void handle_obstacle_avoidance() {
    double distance = read_ultrasonic(US1_TRIG_PIN, US1_ECHO_PIN);

    if (distance < DISTANCE_MIN) {
        std::cout << " | WARNING: Obstacle detected at " << distance << " cm!" << std::endl;
        // Simple stop and reverse
        set_motor_speed(M1_PWM_PIN, 0);
        set_motor_speed(M2_PWM_PIN, 0);
        sleep(1);
        set_motor_direction(M1_DIR_PIN, false); // Reverse
        set_motor_direction(M2_DIR_PIN, false);
        set_motor_speed(M1_PWM_PIN, SPEED_FAST);
        set_motor_speed(M2_PWM_PIN, SPEED_FAST);
        sleep(1);
        set_motor_speed(M1_PWM_PIN, 0);
        set_motor_speed(M2_PWM_PIN, 0);
        std::cout << " | ACTION: Stopped and reversed to avoid obstacle." << std::endl;
    } else {
        std::cout << " | STATUS: Path is clear." << std::endl;
        // Optionally put motion logic here, e.g., resume forward movement
        // set_motor_direction(M1_DIR_PIN, true);
        // set_motor_direction(M2_DIR_PIN, true);
        // set_motor_speed(M1_PWM_PIN, SPEED_MODERATE);
        // set_motor_speed(M2_PWM_PIN, SPEED_MODERATE);
    }
}

/**
 * Placeholder for reading soil moisture using I2C ADC.
 */
int read_soil_moisture() {
    if (soil_i2c_handle < 0) return -1; // Error
    // This is highly dependent on the ADC chip (e.g., ADS1115).
    // Assuming a simple 10-bit or 12-bit return value.
    // In a real application, you'd write a configuration register and then read the data register.
    // For now, return a mock value for demonstration.
    static int mock_moisture = 750;
    mock_moisture += (rand() % 100) - 50; // Jitter
    if (mock_moisture > 900) mock_moisture = 900;
    if (mock_moisture < 100) mock_moisture = 100;
    return mock_moisture;
}

/**
 * Placeholder for reading temperature from an I2C sensor.
 */
double read_temperature() {
    if (temp_i2c_handle < 0) return -99.0; // Error
    // This is highly dependent on the sensor (e.g., BMP280, BME280).
    // In a real application, you'd perform a series of I2C reads and calculations.
    // For now, return a mock value for demonstration.
    static double mock_temp = 25.5;
    mock_temp += (rand() % 20 - 10) / 10.0; // Jitter +/- 1.0 C
    if (mock_temp > 30.0) mock_temp = 30.0;
    if (mock_temp < 20.0) mock_temp = 20.0;
    return mock_temp;
}