#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ArmTFBroadcaster : public rclcpp::Node
{
  public:
    ArmTFBroadcaster()
    : Node("ArmTFBroadcaster")
    {
      shoulder_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "shoulder_pose", 10, std::bind(&ArmTFBroadcaster::shoulder_callback, this, std::placeholders::_1));
      shoulder_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      elbow_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "elbow_pose", 10, std::bind(&ArmTFBroadcaster::elbow_callback, this, std::placeholders::_1));
      elbow_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); 
      wrist_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "wrist_pose", 10, std::bind(&ArmTFBroadcaster::wrist_callback, this, std::placeholders::_1));
      wrist_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); 
      hand_tf_broadcaster_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);       
    }

  private:
    void shoulder_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Shoulder orientation: '%f %f'", msg->pose.orientation.x, msg->pose.orientation.y);

        geometry_msgs::msg::TransformStamped shoulder_tf_msg;
        shoulder_tf_msg.header.stamp = this->get_clock()->now();
        shoulder_tf_msg.header.frame_id = "world";
        shoulder_tf_msg.child_frame_id = "shoulder";

        shoulder_tf_msg.transform.translation.x = 0.0;
        shoulder_tf_msg.transform.translation.y = 0.0;
        shoulder_tf_msg.transform.translation.z = 0.6;
        // TODO: listen too imu for orientation
        shoulder_tf_msg.transform.rotation.x = msg->pose.orientation.x;
        shoulder_tf_msg.transform.rotation.y = msg->pose.orientation.y;
        shoulder_tf_msg.transform.rotation.z = msg->pose.orientation.z;
        shoulder_tf_msg.transform.rotation.w = msg->pose.orientation.w;

        // Send the transformation
        shoulder_tf_broadcaster_->sendTransform(shoulder_tf_msg);
    }

    void elbow_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Elbow orientation: '%f %f'", msg->pose.orientation.x, msg->pose.orientation.y);

        geometry_msgs::msg::TransformStamped elbow_tf_msg;
        elbow_tf_msg.header.stamp = this->get_clock()->now();
        elbow_tf_msg.header.frame_id = "shoulder";
        elbow_tf_msg.child_frame_id = "elbow";

        elbow_tf_msg.transform.translation.x = 0.4;
        elbow_tf_msg.transform.translation.y = 0.0;
        elbow_tf_msg.transform.translation.z = 0.0;
        // TODO: listen IMU for orientation(relative to world not to shoulder)
        elbow_tf_msg.transform.rotation.x = msg->pose.orientation.x;
        elbow_tf_msg.transform.rotation.y = msg->pose.orientation.y;
        elbow_tf_msg.transform.rotation.z = msg->pose.orientation.z;
        elbow_tf_msg.transform.rotation.w = msg->pose.orientation.w;

        // Send the transformation
        elbow_tf_broadcaster_->sendTransform(elbow_tf_msg);
    }

    void wrist_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
    {
        RCLCPP_INFO(this->get_logger(), "Wrist orientation: '%f %f'", msg->pose.orientation.x, msg->pose.orientation.y);
        // wrist transformation relative to elbow
        // Assume hand can not rotate through x,z axis (no twsting wrist, no shake hand movement)
        geometry_msgs::msg::TransformStamped wrist_tf_msg;
        wrist_tf_msg.header.stamp = this->get_clock()->now();
        wrist_tf_msg.header.frame_id = "elbow";
        wrist_tf_msg.child_frame_id = "wrist";

        wrist_tf_msg.transform.translation.x = 0.3;
        wrist_tf_msg.transform.translation.y = 0.0;
        wrist_tf_msg.transform.translation.z = 0.0;

        wrist_tf_msg.transform.rotation.x = 0.0;
        wrist_tf_msg.transform.rotation.y = msg->pose.orientation.y;
        wrist_tf_msg.transform.rotation.z = 0.0;
        wrist_tf_msg.transform.rotation.w = msg->pose.orientation.w;

        // Send the wrist transformation
        wrist_tf_broadcaster_->sendTransform(wrist_tf_msg);

        // Hand transformation relative to wrist
        // Hand transformation to wrist only has translation on x
        geometry_msgs::msg::TransformStamped hand_tf_msg;
        hand_tf_msg.header.stamp = this->get_clock()->now();
        hand_tf_msg.header.frame_id = "wrist";
        hand_tf_msg.child_frame_id = "hand";

        hand_tf_msg.transform.translation.x = 0.1;
        hand_tf_msg.transform.translation.y = 0.0;
        hand_tf_msg.transform.translation.z = 0.0;

        hand_tf_msg.transform.rotation.x = 0.0;
        hand_tf_msg.transform.rotation.y = 0.0;
        hand_tf_msg.transform.rotation.z = 0.0;
        hand_tf_msg.transform.rotation.w = 1.0;
        // Send hand transformation
        hand_tf_broadcaster_->sendTransform(hand_tf_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr shoulder_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr elbow_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wrist_subscription_;    
    std::unique_ptr<tf2_ros::TransformBroadcaster> shoulder_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> elbow_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> wrist_tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> hand_tf_broadcaster_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
