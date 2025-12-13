#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace UNITREE_LEGGED_SDK;

class TeleopNode : public rclcpp::Node {
public:
    TeleopNode() : Node("teleop_go1_node") {
        pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::HighCmd>("high_cmd", 1);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // 50Hz
            std::bind(&TeleopNode::publish_command, this));
        
        // Initialize command message
        initializeCommand();
        
        // Set up non-blocking keyboard input
        setupKeyboard();
        
        RCLCPP_INFO(this->get_logger(), "Unitree Go1 Teleop Node Started");
        RCLCPP_INFO(this->get_logger(), "Controls:");
        RCLCPP_INFO(this->get_logger(), "  w/s: Forward/Backward");
        RCLCPP_INFO(this->get_logger(), "  a/d: Left/Right");
        RCLCPP_INFO(this->get_logger(), "  q/e: Turn Left/Right");
        RCLCPP_INFO(this->get_logger(), "  r/f: Body Up/Down");
        RCLCPP_INFO(this->get_logger(), "  t/g: Pitch Up/Down");
        RCLCPP_INFO(this->get_logger(), "  y/h: Roll Left/Right");
        RCLCPP_INFO(this->get_logger(), "  1: Stand Mode");
        RCLCPP_INFO(this->get_logger(), "  2: Walk Mode");
        RCLCPP_INFO(this->get_logger(), "  0: Idle Mode");
        RCLCPP_INFO(this->get_logger(), "  x: Stop/Zero all velocities");
        RCLCPP_INFO(this->get_logger(), "  ESC: Exit");
    }
    
    ~TeleopNode() {
        restoreKeyboard();
    }

private:
    void initializeCommand() {
        high_cmd_ros_.head[0] = 0xFE;
        high_cmd_ros_.head[1] = 0xEF;
        high_cmd_ros_.level_flag = 0;
        high_cmd_ros_.mode = 1; // Start in stand mode
        high_cmd_ros_.gait_type = 1;
        high_cmd_ros_.speed_level = 0;
        high_cmd_ros_.foot_raise_height = 0.08;
        high_cmd_ros_.body_height = 0.0;
        high_cmd_ros_.euler[0] = 0;
        high_cmd_ros_.euler[1] = 0;
        high_cmd_ros_.euler[2] = 0;
        high_cmd_ros_.velocity[0] = 0.0f;
        high_cmd_ros_.velocity[1] = 0.0f;
        high_cmd_ros_.yaw_speed = 0.0f;
        high_cmd_ros_.reserve = 0;
        
        // Movement parameters
        linear_vel_step_ = 0.1f;
        angular_vel_step_ = 0.5f;
        euler_step_ = 0.1f;
        body_height_step_ = 0.05f;
        max_linear_vel_ = 0.8f;
        max_angular_vel_ = 2.0f;
        max_euler_ = 0.3f;
        max_body_height_ = 0.3f;
        min_body_height_ = -0.3f;
    }
    
    void setupKeyboard() {
        tcgetattr(STDIN_FILENO, &old_termios_);
        struct termios new_termios = old_termios_;
        new_termios.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
        fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
    }
    
    void restoreKeyboard() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
    
    int getKey() {
        int key = getchar();
        return key;
    }
    
    void publish_command() {
        int key = getKey();
        
        if (key != EOF) {
            processKey(key);
        }
        
        // Limit values to safe ranges
        limitValues();
        
        // Publish command
        pub_->publish(high_cmd_ros_);
    }
    
    void processKey(int key) {
        switch(key) {
            // Movement controls
            case 'w': // Forward
                high_cmd_ros_.velocity[0] = std::min(high_cmd_ros_.velocity[0] + linear_vel_step_, max_linear_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Forward: %.2f", high_cmd_ros_.velocity[0]);
                break;
            case 's': // Backward
                high_cmd_ros_.velocity[0] = std::max(high_cmd_ros_.velocity[0] - linear_vel_step_, -max_linear_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Backward: %.2f", high_cmd_ros_.velocity[0]);
                break;
            case 'a': // Left
                high_cmd_ros_.velocity[1] = std::min(high_cmd_ros_.velocity[1] + linear_vel_step_, max_linear_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Left: %.2f", high_cmd_ros_.velocity[1]);
                break;
            case 'd': // Right
                high_cmd_ros_.velocity[1] = std::max(high_cmd_ros_.velocity[1] - linear_vel_step_, -max_linear_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Right: %.2f", high_cmd_ros_.velocity[1]);
                break;
            case 'q': // Turn left
                high_cmd_ros_.yaw_speed = std::min(high_cmd_ros_.yaw_speed + angular_vel_step_, max_angular_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Turn Left: %.2f", high_cmd_ros_.yaw_speed);
                break;
            case 'e': // Turn right
                high_cmd_ros_.yaw_speed = std::max(high_cmd_ros_.yaw_speed - angular_vel_step_, -max_angular_vel_);
                high_cmd_ros_.mode = 2; // Walk mode
                RCLCPP_INFO(this->get_logger(), "Turn Right: %.2f", high_cmd_ros_.yaw_speed);
                break;
                
            // Body position controls
            case 'r': // Body up
                high_cmd_ros_.body_height = std::min(high_cmd_ros_.body_height + body_height_step_, max_body_height_);
                RCLCPP_INFO(this->get_logger(), "Body Height: %.2f", high_cmd_ros_.body_height);
                break;
            case 'f': // Body down
                high_cmd_ros_.body_height = std::max(high_cmd_ros_.body_height - body_height_step_, min_body_height_);
                RCLCPP_INFO(this->get_logger(), "Body Height: %.2f", high_cmd_ros_.body_height);
                break;
            case 't': // Pitch up
                high_cmd_ros_.euler[1] = std::min(high_cmd_ros_.euler[1] + euler_step_, max_euler_);
                RCLCPP_INFO(this->get_logger(), "Pitch: %.2f", high_cmd_ros_.euler[1]);
                break;
            case 'g': // Pitch down
                high_cmd_ros_.euler[1] = std::max(high_cmd_ros_.euler[1] - euler_step_, -max_euler_);
                RCLCPP_INFO(this->get_logger(), "Pitch: %.2f", high_cmd_ros_.euler[1]);
                break;
            case 'y': // Roll left
                high_cmd_ros_.euler[0] = std::min(high_cmd_ros_.euler[0] + euler_step_, max_euler_);
                RCLCPP_INFO(this->get_logger(), "Roll: %.2f", high_cmd_ros_.euler[0]);
                break;
            case 'h': // Roll right
                high_cmd_ros_.euler[0] = std::max(high_cmd_ros_.euler[0] - euler_step_, -max_euler_);
                RCLCPP_INFO(this->get_logger(), "Roll: %.2f", high_cmd_ros_.euler[0]);
                break;
                
            // Mode controls
            case '0': // Idle mode
                high_cmd_ros_.mode = 0;
                RCLCPP_INFO(this->get_logger(), "Mode: Idle");
                break;
            case '1': // Stand mode
                high_cmd_ros_.mode = 1;
                RCLCPP_INFO(this->get_logger(), "Mode: Stand");
                break;
            case '2': // Walk mode
                high_cmd_ros_.mode = 2;
                RCLCPP_INFO(this->get_logger(), "Mode: Walk");
                break;
                
            // Stop all
            case 'x': // Stop
                high_cmd_ros_.velocity[0] = 0.0f;
                high_cmd_ros_.velocity[1] = 0.0f;
                high_cmd_ros_.yaw_speed = 0.0f;
                high_cmd_ros_.euler[0] = 0.0f;
                high_cmd_ros_.euler[1] = 0.0f;
                high_cmd_ros_.euler[2] = 0.0f;
                high_cmd_ros_.body_height = 0.0f;
                RCLCPP_INFO(this->get_logger(), "STOP - All velocities zeroed");
                break;
                
            case 27: // ESC key
                RCLCPP_INFO(this->get_logger(), "Exiting...");
                rclcpp::shutdown();
                break;
        }
    }
    
    void limitValues() {
        // Limit linear velocities
        high_cmd_ros_.velocity[0] = std::max(-max_linear_vel_, std::min(max_linear_vel_, high_cmd_ros_.velocity[0]));
        high_cmd_ros_.velocity[1] = std::max(-max_linear_vel_, std::min(max_linear_vel_, high_cmd_ros_.velocity[1]));
        
        // Limit angular velocity
        high_cmd_ros_.yaw_speed = std::max(-max_angular_vel_, std::min(max_angular_vel_, high_cmd_ros_.yaw_speed));
        
        // Limit euler angles
        high_cmd_ros_.euler[0] = std::max(-max_euler_, std::min(max_euler_, high_cmd_ros_.euler[0]));
        high_cmd_ros_.euler[1] = std::max(-max_euler_, std::min(max_euler_, high_cmd_ros_.euler[1]));
        high_cmd_ros_.euler[2] = std::max(-max_euler_, std::min(max_euler_, high_cmd_ros_.euler[2]));
        
        // Limit body height
        high_cmd_ros_.body_height = std::max(min_body_height_, std::min(max_body_height_, high_cmd_ros_.body_height));
    }
    
    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::HighCmd>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    ros2_unitree_legged_msgs::msg::HighCmd high_cmd_ros_;
    
    // Control parameters
    float linear_vel_step_;
    float angular_vel_step_;
    float euler_step_;
    float body_height_step_;
    float max_linear_vel_;
    float max_angular_vel_;
    float max_euler_;
    float max_body_height_;
    float min_body_height_;
    
    // Terminal settings
    struct termios old_termios_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    std::cout << "WARNING: Control level is set to HIGH-level." << std::endl
              << "Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    auto node = std::make_shared<TeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
