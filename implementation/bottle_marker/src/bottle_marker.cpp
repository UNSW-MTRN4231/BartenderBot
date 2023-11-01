#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "std_msgs/msg/string.hpp"

#include "vision_ros_msgs/msg/bounding_boxes.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class bottle_marker : public rclcpp::Node {
    
public:
    bottle_marker() : Node("bottle_marker") {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bottle_markers", 10);
        ppublisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("detect_bac", 10);
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>
        (
            "detect_bac",10, std::bind(&bottle_marker::publishMarkers, this, _1)
        );

         timer = this->create_wall_timer(500ms, std::bind(&bottle_marker::publishPoses, this));
    };

private:

    void publishPoses() {
        auto poses = geometry_msgs::msg::PoseArray();

        for (int i = 0; i < 4; i++) {
             auto pose = geometry_msgs::msg::Pose();

             pose.position.x = i*1;
             pose.position.y = i*1;
             pose.position.z = i*1;

             poses.poses.push_back(pose);
        }

        ppublisher_->publish(poses);

    } 

    void publishMarkers(const geometry_msgs::msg::PoseArray &heard_msg) {
        std::cout << heard_msg.header.frame_id;

        visualization_msgs::msg::MarkerArray marker_message;

        for (int i=0; i < heard_msg.poses.size(); i++) {
            marker_message.markers.push_back(addMarker(heard_msg.poses[i],i));
        }
        
        RCLCPP_INFO(this->get_logger(),"PUBLISHING POSES");
        publisher_->publish(marker_message);
    };  

    visualization_msgs::msg::Marker addMarker(const geometry_msgs::msg::Pose &pose, int i) {
        auto message = visualization_msgs::msg::Marker();
        message.header.frame_id = "map";
        message.header.stamp = this->now();
        message.ns = "basic_shapes";
        message.id = i;
        message.type = visualization_msgs::msg::Marker::CYLINDER;
        message.action = visualization_msgs::msg::Marker::ADD;
        message.pose.position.x = pose.position.x;
        message.pose.position.y = pose.position.y;
        message.pose.position.z = pose.position.z;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        message.pose.orientation.w = 1.0;
        message.scale.x = 0.2;
        message.scale.y = 0.2;
        message.scale.z = 0.3;
        message.color.r = 1.0f;
        message.color.g = 0.0f;
        message.color.b = 0.0f;
        message.color.a = 1.0;
        return message;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ppublisher_;
    rclcpp::Subscription<vision_ros_msgs::msg::BoundingBoxes>::SharedPtr real_sub_;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bottle_marker>());
  rclcpp::shutdown();
  return 0;
}