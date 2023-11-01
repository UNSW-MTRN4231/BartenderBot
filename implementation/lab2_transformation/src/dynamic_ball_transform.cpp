#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "vision_ros_msgs/msg/bounding_boxes.hpp"
#include <visualization_msgs/msg/marker.hpp> // Include the marker message

class DynamicTFBroadcaster : public rclcpp::Node
{
  public:
    DynamicTFBroadcaster()
    : Node("DynamicTFBroadcaster")
    {
      subscription_ = this->create_subscription<vision_ros_msgs::msg::BoundingBoxes>(
        "detect_bac", 10, std::bind(&DynamicTFBroadcaster::topic_callback, this, std::placeholders::_1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("ball_marker", 10); // Create a marker publisher
    }

  private:
    void topic_callback(const vision_ros_msgs::msg::BoundingBoxes::SharedPtr mesg)
    {
      auto msg = mesg->bounding_boxes[0];
      RCLCPP_INFO(this->get_logger(), "Received ball_pose: x=%f, y=%f", msg.x, msg.y);

      geometry_msgs::msg::TransformStamped transform_msg;
      transform_msg.header.stamp = this->get_clock()->now();
      transform_msg.header.frame_id = "camera_frame";
      transform_msg.child_frame_id = "ball_frame";
      transform_msg.transform.translation.x = msg.x;
      transform_msg.transform.translation.y = msg.y;
      //transform_msg.transform.translation.z = msg->pose.position.z;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0); // Set the rotation (identity in this case)
      transform_msg.transform.rotation.x = q.x();
      transform_msg.transform.rotation.y = q.y();
      transform_msg.transform.rotation.z = q.z();
      transform_msg.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(transform_msg);

      // Create a marker in ball_pose
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "ball_frame"; // Use the same frame as the child frame_id
      marker.header.stamp = this->get_clock()->now();
      marker.ns = "ball_marker";
      marker.id = 0;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = 0.0; // Set marker position relative to the ball frame
      marker.pose.position.y = 0.0;
      marker.pose.position.z = 0.0;
      marker.scale.x = 0.1; // Set the marker scale
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
      marker.color.r = 1.0; // Set marker color (red)
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0; // Set marker alpha (fully opaque)

      marker_publisher_->publish(marker); // Publish the marker
    }

    rclcpp::Subscription<vision_ros_msgs::msg::BoundingBoxes>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
