#include <iostream>
#include <pigpio.h>
#include <unistd.h> // For sleep
#include <iomanip>  // For setprecision
#include <cmath>    // For M_PI, etc. if needed

// --- System Configuration Constants ---

// The three motors will be controlled as follows:
// Motor 1: Hardware PWM (GPIO 18) for speed, Digital Pin (GPIO 23) for direction.
// Motor 2: Software PWM (GPIO 12) for speed, Digital Pin (GPIO 24) for direction.
// Motor 3: Software PWM (GPIO 13) for speed, Digital Pin (GPIO 25) for direction.

// --- Motor Control Pin Definitions (BCM Numbering) ---

// Motor 1 (Hardware PWM)
#define M1_PWM_PIN    18 // Pin 12 - Hardware PWM0 for speed
#define M1_DIR_PIN    23 // Pin 16 - Digital output for direction

// Motor 2 (Software PWM)
#define M2_PWM_PIN    12 // Pin 32 - Software PWM for speed
#define M2_DIR_PIN    24 // Pin 18 - Digital output for direction

// Motor 3 (Software PWM)
#define M3_PWM_PIN    13 // Pin 33 - Software PWM for speed
#define M3_DIR_PIN    25 // Pin 22 - Digital output for direction

// --- Ultrasonic Sensor Definitions (3 Sensors) ---

// Sensor 1 (Front/Left)
#define US1_TRIG_PIN  5  // Pin 29 - Output
#define US1_ECHO_PIN  6  // Pin 31 - Input
// Sensor 2 (Center)
#define US2_TRIG_PIN  16 // Pin 36 - Output
#define US2_ECHO_PIN  20 // Pin 38 - Input
// Sensor 3 (Back/Right)
#define US3_TRIG_PIN  21 // Pin 40 - Output
#define US3_ECHO_PIN  26 // Pin 37 - Input

// --- I2C Sensor Definitions (Temperature and Soil Probe) ---
#define I2C_BUS       1    // Raspberry Pi's standard I2C bus (Pins 3 & 5)

// Sensor 1: Temperature Sensor (BMP280/Placeholder)
#define TEMP_I2C_ADDRESS   0x76 // Example I2C address for Temperature Sensor
int temp_i2c_handle = -1; 

// Sensor 2: Soil Moisture/Probe (Assuming I2C interface, e.g., external ADC or I2C sensor)
#define SOIL_I2C_ADDRESS   0x48 // Example I2C address for a common ADC chip
int soil_i2c_handle = -1;

// --- Control Constants ---
#define SPEED_MAX      255 // Max duty cycle for PWM
#define SPEED_MODERATE 120 
#define TEMP_THRESHOLD 25.0 // Celsius
#define DISTANCE_MIN   20.0  // cm
#define SOIL_WET_LEVEL 500  // Example analog reading threshold

// --- Function Prototypes ---
void setup_gpios();
void cleanup_gpios();

void set_motor_speed(int pwm_pin, int speed);
void set_motor_direction(int dir_pin, bool forward);

float read_temperature();
int read_soil_moisture(); // Returns a raw integer reading
float read_ultrasonic_distance(int trig_pin, int echo_pin);

#include <iostream>


--- Main Program ---
int main(int argc, char *argv[]) {
    std::cout << "Starting PiGPIO Complex Controller..." << std::endl;

    if (gpioInitialise() < 0) {
        std::cerr << "PiGPIO initialization failed. Ensure pigpiod is running and you have privileges (sudo)." << std::endl;
        return 1;
    }

    std::cout << "PiGPIO initialized successfully." << std::endl;
    setup_gpios(); // Initialize all pins and I2C connections

    // --- Complex Control Sequence (10 Cycles) ---
    std::cout << "\nStarting 10-cycle environmental control loop..." << std::endl;

    for (int i = 0; i < 10; ++i) {
        float temp = read_temperature();
        int soil = read_soil_moisture();
        float dist1 = read_ultrasonic_distance(US1_TRIG_PIN, US1_ECHO_PIN);

        std::cout << std::fixed << std::setprecision(2);
        std::cout << "--- Cycle " << i + 1 << " ---" << std::endl;
        std::cout << " Temp: " << temp << "°C | Soil: " << soil << " (Threshold: " << SOIL_WET_LEVEL << ")" << std::endl;
        std::cout << " Dist 1: " << dist1 << " cm | ";
        std::cout << " Dist 2: " << read_ultrasonic_distance(US2_TRIG_PIN, US2_ECHO_PIN) << " cm | ";
        std::cout << " Dist 3: " << read_ultrasonic_distance(US3_TRIG_PIN, US3_ECHO_PIN) << " cm" << std::endl;

        // --- Logic 1: Temperature Control (Motor 1) ---
        if (temp > TEMP_THRESHOLD) {
            std::cout << " [M1] Overheat: Running FORWARD to cool." << std::endl;
            set_motor_direction(M1_DIR_PIN, true); 
            set_motor_speed(M1_PWM_PIN, SPEED_MODERATE);
        } else {
            set_motor_speed(M1_PWM_PIN, 0);
        }
        
        // --- Logic 2: Obstacle Avoidance (Motor 2 & 3) ---
        if (dist1 < DISTANCE_MIN) {
            std::cout << " [M2/M3] Obstacle detected! Running REVERSE." << std::endl;
            set_motor_direction(M2_DIR_PIN, false);
            set_motor_direction(M3_DIR_PIN, false);
            set_motor_speed(M2_PWM_PIN, SPEED_FAST);
            set_motor_speed(M3_PWM_PIN, SPEED_FAST);
        } else {
             set_motor_speed(M2_PWM_PIN, 0);
             set_motor_speed(M3_PWM_PIN, 0);
        }

        sleep(3); // Wait 3 seconds before the next check
    }

    std::cout << "Control sequence complete. Cleaning up..." << std::endl;
    cleanup_gpios(); // Clean up all pins and terminate pigpio
    return 0;
}



// --- Setup and Cleanup Functions ---

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
    
    // M3 (SW PWM)
    gpioSetMode(M3_PWM_PIN, PI_OUTPUT);
    gpioSetMode(M3_DIR_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(M3_PWM_PIN, 500); // Set software PWM frequency
    gpioSetPWMrange(M3_PWM_PIN, SPEED_MAX);

    // Initial stop for all motors
    set_motor_speed(M1_PWM_PIN, 0);
    set_motor_speed(M2_PWM_PIN, 0);
    set_motor_speed(M3_PWM_PIN, 0);

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
    // Close I2C handles
    if (temp_i2c_handle >= 0) i2cClose(temp_i2c_handle);
    if (soil_i2c_handle >= 0) i2cClose(soil_i2c_handle);

    // Set all used GPIOs back to input for safety
    int pins[] = {M1_PWM_PIN, M1_DIR_PIN, M2_PWM_PIN, M2_DIR_PIN, M3_PWM_PIN, M3_DIR_PIN,
                  US1_TRIG_PIN, US1_ECHO_PIN, US2_TRIG_PIN, US2_ECHO_PIN, US3_TRIG_PIN, US3_ECHO_PIN};
    for (int pin : pins) {
        gpioSetMode(pin, PI_INPUT);
    }

    // Terminate the pigpio library
    gpioTerminate();
    std::cout << "PiGPIO terminated. Program finished." << std::endl;
}

// --- Motor Helper Functions ---

/**
 * @brief Sets the motor speed using PWM (either hardware or software).
 * @param pwm_pin The GPIO pin defined as the PWM output.
 * @param speed Duty cycle from 0 (off) to SPEED_MAX (full speed).
 */
void set_motor_speed(int pwm_pin, int speed) {
    // Use gpioPWM for both hardware and software PWM (pigpio handles the difference)
    gpioPWM(pwm_pin, speed);
}

/**
 * @brief Sets the motor direction by toggling a digital pin.
 * @param dir_pin The GPIO pin defined as the direction output.
 * @param forward true for one direction (e.g., HIGH), false for the other (e.g., LOW).
 */
void set_motor_direction(int dir_pin, bool forward) {
    int level = forward ? 1 : 0;
    gpioWrite(dir_pin, level);
}

// --- Sensor Helper Functions ---

/**
 * @brief Reads a dummy temperature value using the I2C handle.
 * @return Temperature in Celsius.
 */
float read_temperature() {
    if (temp_i2c_handle < 0) return 20.0; // Return a safe default if I2C failed

    // --- Placeholder logic for demonstration ---
    static float current_simulated_temp = 20.0; 
    current_simulated_temp += 0.5; 
    if (current_simulated_temp > 30.0) { current_simulated_temp = 20.0; }
    return current_simulated_temp;
}

/**
 * @brief Reads a dummy soil moisture value using the I2C handle.
 * @return Raw integer reading (simulated ADC value).
 */
int read_soil_moisture() {
    if (soil_i2c_handle < 0) return 300; // Return a safe default

    // --- Placeholder logic for demonstration ---
    // Simulate a moisture reading that fluctuates
    static int current_simulated_soil = 300; 
    if (rand() % 2 == 0) {
        current_simulated_soil += 50;
    } else {
        current_simulated_soil -= 50;
    }
    if (current_simulated_soil < 100) current_simulated_soil = 700;
    if (current_simulated_soil > 800) current_simulated_soil = 300;

    return current_simulated_soil;
}

/**
 * @brief Reads the distance from an ultrasonic sensor (HC-SR04).
 * Uses gpioTrigger to send a pulse and waits for the echo pulse.
 * @param trig_pin The GPIO pin connected to the TRIG pin of the sensor.
 * @param echo_pin The GPIO pin connected to the ECHO pin of the sensor.
 * @return Distance in centimeters (cm).
 */
float read_ultrasonic_distance(int trig_pin, int echo_pin) {
    // Distance = (Time of Flight * Speed of Sound) / 2
    // Speed of sound = 34300 cm/s or 34.3 cm/us
    const float SOUND_SPEED_US_CM = 0.0343f; 
    
    // Send a 10us pulse to the trigger pin
    gpioTrigger(trig_pin, 10, 1); 

    // Wait for the echo pin to go HIGH (start of pulse)
    long start_time = gpioTick();
    while (gpioRead(echo_pin) == 0 && (gpioTick() - start_time) < 50000) { /* timeout */ }

    long pulse_start = gpioTick();

    // Wait for the echo pin to go LOW (end of pulse)
    long timeout_start = gpioTick();
    while (gpioRead(echo_pin) == 1 && (gpioTick() - timeout_start) < 50000) { /* timeout */ }

    long pulse_end = gpioTick();

    // Calculate pulse width (duration) in microseconds
    long pulse_duration = pulse_end - pulse_start;

    // Convert duration to distance in cm
    // Division by 2 accounts for the signal traveling out and back.
    float distance_cm = (float)pulse_duration * SOUND_SPEED_US_CM / 2.0f; 

    // Filter out obvious noise/bad readings
    if (distance_cm > 400.0f || distance_cm < 2.0f) {
        return -1.0f; // Indicates a measurement error or out of range
    }

    return distance_cm;
}
