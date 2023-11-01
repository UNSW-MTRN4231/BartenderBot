#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include "vision_ros_msgs/msg/bounding_boxes.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class DynamicTFBroadcaster : public rclcpp::Node
{
  public:
    DynamicTFBroadcaster()
    : Node("DynamicTFBroadcaster")
    {
      subscription_ = this->create_subscription<vision_ros_msgs::msg::BoundingBoxes>(
        "detect_bac", 10, std::bind(&DynamicTFBroadcaster::topic_callback, this, std::placeholders::_1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  private:
    void topic_callback(const vision_ros_msgs::msg::BoundingBoxes::SharedPtr mesg)
    {
      auto msg = mesg->bounding_boxes[0];
      RCLCPP_INFO(this->get_logger(), "I heard: x=%f, y=%f", msg.x, msg.y);

      geometry_msgs::msg::TransformStamped transform_msg;
      //transform_msg.header.stamp = this->get_clock()->now();
      transform_msg.header.frame_id = "camera_frame";
      transform_msg.child_frame_id = "bottle_blue";
      transform_msg.transform.translation.x = msg.x;
      transform_msg.transform.translation.y = msg.y;
      transform_msg.transform.translation.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0); // Set the rotation (identity in this case)
      transform_msg.transform.rotation.x = q.x();
      transform_msg.transform.rotation.y = q.y();
      transform_msg.transform.rotation.z = q.z();
      transform_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform_msg);
    }

    rclcpp::Subscription<vision_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
