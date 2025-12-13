#include "rclcpp/rclcpp.hpp"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/imu.hpp"

class ImuPublisherNode : public rclcpp::Node {
public:
    ImuPublisherNode() : Node("imu_publisher_node") {
        imu_pub_ = this->create_publisher<ros2_unitree_legged_msgs::msg::IMU>("imu/data_raw", 10);
        high_state_sub_ = this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>(
            "high_state",
            10,
            std::bind(&ImuPublisherNode::highStateCallback, this, std::placeholders::_1));
    }

private:
    void highStateCallback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg) {
        // Debug print for IMU quaternion z component
        RCLCPP_INFO(this->get_logger(), "Received IMU quaternion z: %f", msg->imu.quaternion[2]);

        // Publish IMU data extracted from HighState
        imu_pub_->publish(msg->imu);
    }

    rclcpp::Publisher<ros2_unitree_legged_msgs::msg::IMU>::SharedPtr imu_pub_;
    rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_sub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto imu_node = std::make_shared<ImuPublisherNode>();
    rclcpp::spin(imu_node);
    rclcpp::shutdown();
    return 0;
}
