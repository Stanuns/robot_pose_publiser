#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

using namespace std::chrono_literals;

class RobotPosePublisher : public rclcpp::Node
{
public:
  RobotPosePublisher()
  : Node("robot_pose_publisher")
  {
    // Declare and acquire `target_frame` parameter
    target_frame_ = this->declare_parameter<std::string>("target_frame", "base_footprint");

    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create robot_pose publisher
    publisher_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 1);

      RCLCPP_INFO(
      this->get_logger(), "robot pose publish........");

    // Call on_timer function every second
    timer_ = this->create_wall_timer(
      0.1s, std::bind(&RobotPosePublisher::on_timer, this));
  }

private:
  void on_timer()
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string fromFrameRel = target_frame_.c_str();
    std::string toFrameRel = "map";


    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and turtle2 frames
    // and send velocity commands for turtle2 to reach target_frame
    rclcpp::Time now = this->get_clock()->now();
    // rclcpp::Time when = now - rclcpp::Duration(1, 0);
    try {
      t = tf_buffer_->lookupTransform(
        toFrameRel, 
        fromFrameRel,
        now,
        50ms); //tf2::TimePointZero
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = fromFrameRel;
    msg.pose.pose.position.x = t.transform.translation.x;
    msg.pose.pose.position.y = t.transform.translation.y;
    msg.pose.pose.position.z = t.transform.translation.z;
    // tf2::Quaternion q;
    msg.pose.pose.orientation = t.transform.rotation;

    publisher_->publish(msg);
    
  }

  rclcpp::TimerBase::SharedPtr timer_{nullptr};
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_{nullptr};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string target_frame_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
