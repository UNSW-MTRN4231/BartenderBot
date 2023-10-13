#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include "brain_msgs/msg/command.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class arm_brain : public rclcpp::Node {
  public:
    arm_brain() : Node("arm_brain")
    {
      // Initalise the brain subscriber
      subscription_ = this->create_subscription<brain_msgs::msg::Command>("command", 10, std::bind(&arm_brain::brain, this, _1));

      // Initialise the pose publisher
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("move_to", 10);

      // Initialise current arm pose
      curr_pose.position.x = 0;
      curr_pose.position.y = 0;
      curr_pose.position.z = 0;
      curr_pose.orientation.x = 0;
      curr_pose.orientation.y = 0;
      curr_pose.orientation.z = 0;
      curr_pose.orientation.w = 0;

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }

  private:
    //Function to generate a target position message
    geometry_msgs::msg::Pose generatePoseFromTransform(geometry_msgs::msg::TransformStamped tf) {
      geometry_msgs::msg::Pose msg;
    
      msg.position.x = tf.transform.translation.x;
      msg.position.y = tf.transform.translation.y;
      msg.position.z = tf.transform.translation.z;
      
      // TODO: CHECK ROTATION OF GRIPPER FOR PICKUPS
      msg.orientation.x = tf.transform.rotation.x;
      msg.orientation.y = tf.transform.rotation.y;
      msg.orientation.z = tf.transform.rotation.z;
      msg.orientation.w = tf.transform.rotation.w;
      
      return msg;
    }
    
    void send_pose() {
      geometry_msgs::msg::Pose pose = curr_pose;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f' '%f' ", pose.position.x, pose.position.y, pose.position.z);
      pose_publisher_->publish(pose);
    }
    // toggle gripper
    void grip() {

    }

    void home() {
      curr_pose.position.x = 234.44;
      curr_pose.position.y = -816.65;
      curr_pose.position.z = -336.17;
      
      tf2::Quaternion q;
      q.setRPY(1.206, 1.206 , 1.206);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();
    }

    void move(geometry_msgs::msg::TransformStamped tf) {
      curr_pose = generatePoseFromTransform(tf);

      send_pose();

      //TODO: CHECK IF OBJECT IS STILL IN LOCATION -> MOVE AGAIN
    }

    // combines both parts of shaker and mixes, then dissasembles
    void shake(geometry_msgs::msg::Pose big, geometry_msgs::msg::Pose small) {
      // picks up small shaker, moves above other, rotates and combines
      curr_pose = small;
      send_pose();
      grip();
      curr_pose.position.z += 20;
      send_pose();
      curr_pose = big;
      curr_pose.position.z += 20;
      send_pose();

      curr_pose.orientation.z -= 10;
      send_pose();
      grip();
      // picks up combined, moves above and rotates

      // places back down and dissasembles


    }
    
    // pours the bottle at location
    void pour(double offset) {
        // moves accross offset width
        curr_pose.position.x += offset;
        send_pose();
        // pours drink
        
        tf2::Quaternion q;
        q.setRPY(M_PI/4, 0 , 0); // 45 degrees roll in rad
        curr_pose.orientation.x = q.x();
        curr_pose.orientation.y = q.y();
        curr_pose.orientation.z = q.z();
        curr_pose.orientation.w = q.w();
        send_pose();
        curr_pose.orientation.x = 0;
        curr_pose.orientation.y = 0;
        curr_pose.orientation.z = 0;
        curr_pose.orientation.w = 0;
        send_pose();

        // returns to original position
        curr_pose.position.x -=offset;
        send_pose();
    }

    

    void brain(const brain_msgs::msg::Command &msg) const {
      /*
      
      //moves to home
      home();
      // check for what command is
      switch(msg.item) {
        
        // adding ingredient
        case "fill":
          //need repeat
          move(); //to bottle
          grip();  
          move(); //to shaker
          pour();
          move(); //to old bottle;
          grip(); //release

          break;
        // mixing ingredients
        case "shake":
          shake();
          break:
        
        // pouring cocktail
        case "pour":
          move(); //to shaker
          grip();
          
          //need repeat
          move();
          pour();

          move() //to old shaker
          grip(); //release

          break;

        default:
      }
      */
    }

    void tfCallback(req_frame) {
     // Check if the transformation is between "world" and "req_frame"
        std::string fromFrameRel = "world";
        std::string toFrameRel = req_frame;

        try {
            t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
            return;
        }
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped t;

    rclcpp::Subscription<brain_msgs::msg::Command>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_publisher_;
    geometry_msgs::msg::Pose curr_pose;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_brain>());
  rclcpp::shutdown();
  return 0;
}