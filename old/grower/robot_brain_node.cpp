#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;

// --- Constants (in ROS standard units - meters, m/s) ---
#define DANGER_DISTANCE  0.20  // 20 cm
#define REVERSE_SPEED   -0.3  // m/s
#define FORWARD_SPEED   0.5   // m/s
#define TURN_SPEED      0.5   // rad/s
#define SOIL_WET_LEVEL  500   // ADC threshold
#define AVOID_DURATION  500ms // Time to reverse/turn

enum RobotState {
    FORWARD,
    REVERSE_AVOID,
    TURN_AVOID,
    WATERING // Placeholder state
};

class RobotBrainNode : public rclcpp::Node
{
public:
    RobotBrainNode() : Node("robot_brain_node")
    {
        // 1. Publisher for motor commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // 2. Subscribers for all sensors
        sub_us1_ = this->create_subscription<sensor_msgs::msg::Range>("ultrasound/front", 10, 
            std::bind(&RobotBrainNode::us_callback, this, std::placeholders::_1, 0));
        sub_us2_ = this->create_subscription<sensor_msgs::msg::Range>("ultrasound/left", 10, 
            std::bind(&RobotBrainNode::us_callback, this, std::placeholders::_1, 1));
        sub_us3_ = this->create_subscription<sensor_msgs::msg::Range>("ultrasound/right", 10, 
            std::bind(&RobotBrainNode::us_callback, this, std::placeholders::_1, 2));
        sub_us4_ = this->create_subscription<sensor_msgs::msg::Range>("ultrasound/rear", 10, 
            std::bind(&RobotBrainNode::us_callback, this, std::placeholders::_1, 3));
            
        sub_soil_ = this->create_subscription<std_msgs::msg::Int32>("sensor/soil_moisture", 10, 
            std::bind(&RobotBrainNode::soil_callback, this, std::placeholders::_1));

        // 3. Main decision timer (Less frequent than sensor reads)
        decision_timer_ = this->create_wall_timer(
            500ms, std::bind(&RobotBrainNode::main_decision_loop, this));

        // 4. Avoidance timer (Active only during recovery maneuvers)
        avoidance_timer_ = this->create_wall_timer(
            AVOID_DURATION, std::bind(&RobotBrainNode::avoidance_recovery_callback, this));
        avoidance_timer_->cancel(); // Start inactive
        
        current_state_ = FORWARD;
        RCLCPP_INFO(this->get_logger(), "RobotBrainNode initialized. State: FORWARD");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sub_us1_, sub_us2_, sub_us3_, sub_us4_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_soil_;
    rclcpp::TimerBase::SharedPtr decision_timer_, avoidance_timer_;

    // Sensor storage (index: 0=front, 1=left, 2=right, 3=rear)
    std::array<double, 4> us_distances_ = {999.0, 999.0, 999.0, 999.0}; 
    int current_soil_moisture_ = 1000;
    RobotState current_state_;

    // --- ROS CALLBACKS ---

    // 1. Ultrasonic Sensor Callback (handles all 4 sensors)
    void us_callback(const sensor_msgs::msg::Range::SharedPtr msg, size_t index)
    {
        us_distances_[index] = msg->range;
    }

    // 2. Soil Moisture Callback
    void soil_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        current_soil_moisture_ = msg->data;
    }

    // 3. Main Decision Loop (runs every 500ms)
    void main_decision_loop()
    {
        // Only run decision logic if not in an active avoidance maneuver
        if (current_state_ == REVERSE_AVOID || current_state_ == TURN_AVOID) {
            return;
        }

        // --- 3a. Obstacle Check (Front) ---
        // Find the minimum distance from front/left/right sensors
        double min_front_dist = *std::min_element(us_distances_.begin(), us_distances_.begin() + 3);

        if (min_front_dist < DANGER_DISTANCE) {
            RCLCPP_WARN(this->get_logger(), "Front Obstacle: %.2f m. Transition to REVERSE_AVOID.", min_front_dist);
            current_state_ = REVERSE_AVOID;
            avoidance_timer_->reset(); // Start reverse timer
            publish_cmd_vel(REVERSE_SPEED, 0.0);
            return;
        }

        // --- 3b. Reverse Restriction Check (Rear) ---
        if (us_distances_[3] < DANGER_DISTANCE) {
            RCLCPP_WARN(this->get_logger(), "Rear Obstacle: %.2f m. Cannot reverse.", us_distances_[3]);
            // If the state is FORWARD, this check prevents a bug where logic might try to reverse 
            // if another sensor sees something, but the rear is blocked.
            if (current_state_ != WATERING) {
                publish_cmd_vel(FORWARD_SPEED * 0.5, 0.0); // Slow down
            }
            // Cannot trigger reverse/turn logic if the rear is blocked!
        }

        // --- 3c. Watering Logic (High-level task) ---
        if (current_soil_moisture_ < SOIL_WET_LEVEL && current_state_ != WATERING) {
            RCLCPP_INFO(this->get_logger(), "Soil dry (%d). Transition to WATERING state.", current_soil_moisture_);
            current_state_ = WATERING;
            publish_cmd_vel(0.0, 0.0); // Stop for watering
            // In a real system, you would activate a pump here.
            RCLCPP_INFO(this->get_logger(), "ACTION: Watering for 5 seconds...");
            // Use a separate timer (not avoidance timer) or sleep for real pump action
            // For now, we'll just wait 5 seconds in the next loop and return to FORWARD.
        }

        // --- 3d. Default State ---
        if (current_state_ == FORWARD) {
            publish_cmd_vel(FORWARD_SPEED, 0.0);
        } else if (current_state_ == WATERING) {
             // In a simplified example, we transition back to FORWARD after one cycle
             current_state_ = FORWARD;
        }
    }

    // 4. Avoidance Recovery Callback (Fires after AVOID_DURATION)
    void avoidance_recovery_callback()
    {
        avoidance_timer_->cancel();
        
        if (current_state_ == REVERSE_AVOID) {
            // Finished reversing, now turn to avoid
            RCLCPP_INFO(this->get_logger(), "Finished reverse. Starting turn to avoid.");
            current_state_ = TURN_AVOID;
            // Determine turn direction: turn right if left is closer (US2) or equal, otherwise turn left.
            double angular_vel = (us_distances_[1] <= us_distances_[2]) ? TURN_SPEED : -TURN_SPEED;
            publish_cmd_vel(0.0, angular_vel);
            avoidance_timer_->reset(); // Start turn timer
        }
        else if (current_state_ == TURN_AVOID) {
            // Finished turning, check again or return to default
            RCLCPP_INFO(this->get_logger(), "Finished turn. Returning to FORWARD state.");
            current_state_ = FORWARD;
            publish_cmd_vel(FORWARD_SPEED, 0.0);
        }
    }

    // Helper function to send motor commands
    void publish_cmd_vel(double linear_x, double angular_z)
    {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = linear_x;
        twist_msg.angular.z = angular_z;
        cmd_vel_publisher_->publish(twist_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published Command: L=%.2f, A=%.2f", linear_x, angular_z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotBrainNode>());
    rclcpp::shutdown();
    return 0;
}