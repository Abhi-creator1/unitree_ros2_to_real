#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "ros2_unitree_legged_msgs/msg/high_state.hpp"

using std::placeholders::_1;

class UnitreeOdomNode : public rclcpp::Node
{
public:
  UnitreeOdomNode() : Node("unitree_odom_node")
  {
    // Publishers
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Subscriptions
    high_state_sub_ = create_subscription<
        ros2_unitree_legged_msgs::msg::HighState>(
        "high_state", 10,
        std::bind(&UnitreeOdomNode::highStateCallback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 50,
        std::bind(&UnitreeOdomNode::imuCallback, this, _1));

    last_time_ = now();
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);

    tf2::Matrix3x3(q).getRPY(roll_, pitch_, yaw_);
    
  }

  void highStateCallback(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg)
  {
    rclcpp::Time current_time = now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    double v = msg->velocity[0];  // forward velocity (m/s)

    // Integrate position
    x_ += v * cos(yaw_) * dt;
    y_ += v * sin(yaw_) * dt;

    publishOdom(current_time, v);
    publishTF(current_time);
  }

  void publishOdom(const rclcpp::Time& time, double v)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v;

    // Set small non-zero covariance so SLAM Toolbox uses odom
    for(int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.01;
        odom.twist.covariance[i] = 0.01;
    }

    odom_pub_->publish(odom);
  }

  void publishTF(const rclcpp::Time& time)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";

    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    tf.transform.rotation.x = q.x();
    tf.transform.rotation.y = q.y();
    tf.transform.rotation.z = q.z();
    tf.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf);
  }

  // Robot state
  double x_{0.0}, y_{0.0};
  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};

  rclcpp::Time last_time_;

  // ROS interfaces
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr high_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UnitreeOdomNode>());
  rclcpp::shutdown();
  return 0;
}
