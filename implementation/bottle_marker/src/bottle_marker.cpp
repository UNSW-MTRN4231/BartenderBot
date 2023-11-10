#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include "std_msgs/msg/string.hpp"

#include "vision_ros_msgs/msg/bounding_boxes.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class bottle_marker : public rclcpp::Node {
    
public:
    bottle_marker() : Node("bottle_marker") {
        publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bottle_markers", 10);
        real_sub_ = this->create_subscription<vision_ros_msgs::msg::BoundingBoxes>
        (
            "detect_bac",10, std::bind(&bottle_marker::publishMarkers, this, _1)
        );

        planning_scene_diff_publisher_ = this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
        
        RCLCPP_INFO(this->get_logger(),"PLANNING SCENE CONNECTION ESTABLISHED");

        primitive.type = primitive.CYLINDER;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = 0.20;
        primitive.dimensions[1] = 0.025;

        //timer = this->create_wall_timer(500ms, std::bind(&bottle_marker::publishPoses, this));
    };

private:

    void publishPoses() {
        auto poses = geometry_msgs::msg::PoseArray();

        for (int i = 0; i < 4; i++) {
             auto pose = geometry_msgs::msg::Pose();

             pose.position.x = 0.25+i*0.15;
             pose.position.y = 0.25+i*0.15;
             pose.position.z = 0.1;

             poses.poses.push_back(pose);
        }

        ppublisher_->publish(poses);

    } 

    void publishMarkers(const vision_ros_msgs::msg::BoundingBoxes &heard_msg) {
        //std::cout << heard_msg.header.frame_id;

        visualization_msgs::msg::MarkerArray marker_message;
        planning_scene.world.collision_objects.clear();
        RCLCPP_INFO(this->get_logger(),"Number of Objects: %i", heard_msg.bounding_boxes.size());
        
        for (int i=0; i < heard_msg.bounding_boxes.size(); i++) {
            marker_message.markers.push_back(addMarker(heard_msg.bounding_boxes[i],i));

            moveit_msgs::msg::AttachedCollisionObject attached_object;
            attached_object.link_name = "world";
            /* The header must contain a valid TF frame*/
            attached_object.object.header.frame_id = "world";
            /* The id of the object */
            attached_object.object.id = "bottle"+i;

            /* A default pose */
            geometry_msgs::msg::Pose pose;
            pose.position.x = heard_msg.bounding_boxes[i].x;
            pose.position.y = heard_msg.bounding_boxes[i].y;
            pose.position.z = heard_msg.bounding_boxes[i].z;
            pose.orientation.w = 1.0;

            /* Define a box to be attached */

            attached_object.object.primitives.push_back(primitive);
            attached_object.object.primitive_poses.push_back(pose);
            planning_scene.world.collision_objects.push_back(attached_object.object);
        }
        
        RCLCPP_INFO(this->get_logger(),"PUBLISHING POSES");
        publisher_->publish(marker_message);
        
        planning_scene.is_diff = true;
        planning_scene_diff_publisher_->publish(planning_scene);
    };  

    visualization_msgs::msg::Marker addMarker(const vision_ros_msgs::msg::BoundingBox &pose, int i) {
        auto message = visualization_msgs::msg::Marker();
        message.header.frame_id = "camera_link";
        message.header.stamp = this->now();
        message.ns = "basic_shapes";
        message.id = i;
        message.type = visualization_msgs::msg::Marker::CYLINDER;
        message.action = visualization_msgs::msg::Marker::ADD;
        message.pose.position.x = pose.x;
        message.pose.position.y = pose.y;
        message.pose.position.z = pose.z;
        message.pose.orientation.x = 0.0;
        message.pose.orientation.y = 0.0;
        message.pose.orientation.z = 0.0;
        message.pose.orientation.w = 1.0;
        message.scale.x = 0.1;
        message.scale.y = 0.1;
        message.scale.z = 0.3;
        message.color.r = 1.0f;
        message.color.g = 0.0f;
        message.color.b = 0.0f;
        message.color.a = 1.0;
        return message;
    }

    moveit_msgs::msg::PlanningScene planning_scene;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr ppublisher_;
    rclcpp::Subscription<vision_ros_msgs::msg::BoundingBoxes>::SharedPtr real_sub_;

    shape_msgs::msg::SolidPrimitive primitive;

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher_;

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bottle_marker>());
  rclcpp::shutdown();
  return 0;
}