#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualisation_msgs/msg/marker_array>

#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class bottle_marker : public rclcpp::Node {
    
public:
    bottle_marker() : Node("bottle_marker") {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bottle_markers", 10);
    }
    

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bottle_marker>());
  rclcpp::shutdown();
  return 0;
}