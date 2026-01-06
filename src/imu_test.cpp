#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

class ImuPublisherNode : public rclcpp::Node
{
public:
  ImuPublisherNode() : Node("unitree_imu_publisher")
  {
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    high_state_sub_ = create_subscription<ros2_unitree_legged_msgs::msg::HighState>(
      "high_state", 10,
      std::bind(&ImuPublisherNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu;

    imu.header.stamp = now();
    imu.header.frame_id = "base_link";

    // Quaternion (FULL, not just z)
    imu.orientation.x = msg->imu.quaternion[0];
    imu.orientation.y = msg->imu.quaternion[1];
    imu.orientation.z = msg->imu.quaternion[2];
    imu.orientation.w = msg->imu.quaternion[3];

    // Angular velocity
    imu.angular_velocity.x = msg->imu.gyroscope[0];
    imu.angular_velocity.y = msg->imu.gyroscope[1];
    imu.angular_velocity.z = msg->imu.gyroscope[2];

    // Linear acceleration
    imu.linear_acceleration.x = msg->imu.accelerometer[0];
    imu.linear_acceleration.y = msg->imu.accelerometer[1];
    imu.linear_acceleration.z = msg->imu.accelerometer[2];

    // Covariances (IMPORTANT)
    for(int i=0;i<9;i++) {
      imu.orientation_covariance[i] = 0.01;
      imu.angular_velocity_covariance[i] = 0.01;
      imu.linear_acceleration_covariance[i] = 0.01;
    }

    imu_pub_->publish(imu);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisherNode>());
  rclcpp::shutdown();
  return 0;
}

