#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "brain_msgs/msg/command.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class test_brain : public rclcpp::Node {
  public:
    test_brain() : Node("test_brain")
    {
      // Initialise the pose publisher
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_to", 10);

      // Initialise the arduino publisher
      arduino_publisher_ = this->create_publisher<std_msgs::msg::String>("arduino", 10);
      //timer
      timer_ = this->create_wall_timer(5000ms, std::bind(&test_brain::brain, this));

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
    
    //sends pose to ur
    void send_pose(std::string type) {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose = curr_pose;
      msg.header.frame_id = type;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f' '%f' ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      pose_publisher_->publish(msg);
      sleep(20.0);
    }

    void send_pose() {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose = curr_pose;
      msg.header.frame_id = "free";
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f' '%f' ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      pose_publisher_->publish(msg);
      sleep(20.0);
    }
    
    // toggle gripper
    void grip(bool toggle) {
      std_msgs::msg::String grip;
      if (toggle == 0) {
        grip.data  = "open";
        toggle = 1;
      }
      else if(toggle == 1) {
        grip.data = "close";
        toggle = 0;
      }
      arduino_publisher_->publish(grip);
    }

    //returns robot to home position
    void home() {
      curr_pose.position.x = 0.58835;
      curr_pose.position.y = 0.133;
      curr_pose.position.z = 0.37212;
      
      tf2::Quaternion q;
      q.setRPY(-M_PI, 0 , M_PI/2);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose("home");
    }

    void pickup(geometry_msgs::msg::Pose pose) {
      tf2::Quaternion q;
      if (pose.position.y > 0.3) {
        curr_pose.position.y = pose.position.y - 0.15;
        q.setRPY(M_PI/2 ,-M_PI/2 , M_PI);
      } else {
        curr_pose.position.y = pose.position.y + 0.15;
        q.setRPY(M_PI, -M_PI/2 , -M_PI/2);
      }
      curr_pose.position.x = pose.position.x;
      curr_pose.position.z = pose.position.z;
      
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();

      curr_pose.position.y = pose.position.y;
      send_pose("linear");
      grip(1);

      curr_pose.position.z = 0.3;
      send_pose("linear");
    }

    // combines both parts of shaker and mixes, then dissasembles
    void shake(geometry_msgs::msg::Pose big, geometry_msgs::msg::Pose small) {
      // picks up small shaker, moves above other, rotates and combines
      
      pickup(small);
      tf2::Quaternion q;
      q.setRPY(M_PI, 0 , M_PI/2);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();

      curr_pose.position.x = big.position.x;
      curr_pose.position.y = big.position.y;
      send_pose("linear");

      curr_pose.position.z -= 0.15;
      send_pose("linear");
      grip(0);
      // picks up combined, moves above and rotates
      curr_pose.position.z -= 0.05;
      send_pose("linear");
      grip(1);

      curr_pose.position.z += 0.3;
      send_pose("linear");

    
      q.setRPY(M_PI, 0 , M_PI/2);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();
      q.setRPY(-M_PI, 0 , M_PI/2);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();

      // places back down and dissasembles
      curr_pose.position.z -= 0.3;
      send_pose("linear");
      grip(0);

      curr_pose.position.z += 0.05;
      send_pose("linear");
      grip(1);

      curr_pose.position.z += 0.10;
      send_pose("linear");

      q.setRPY(M_PI, 0 , M_PI/2);
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();

      curr_pose.position.x = small.position.x;
      curr_pose.position.y = small.position.y;
      send_pose("linear");
      curr_pose.position.z = small.position.z;
      send_pose("linear");
      grip(0);
    }
    
    // pours the bottle at current location
    void pour(float offset) {
      offset = offset/2/sqrt(2);
        
      tf2::Quaternion q;
      q.setRPY(-M_PI/2, -M_PI/2 , M_PI/2); // facing computer
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();

      if (curr_pose.position.y > 0.3) {
        // moves accross offset width
        curr_pose.position.y = curr_pose.position.y - offset;
        send_pose("linear");
        q.setRPY(-M_PI/2, M_PI/4 , M_PI/2); // pour towards
      } else {
        // moves accross offset width
        curr_pose.position.y = curr_pose.position.y + offset;
        send_pose("linear");
        q.setRPY(M_PI/2, M_PI/4 , -M_PI/2); // pour away
      }
      // pours drink
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();
      sleep(2);


      q.setRPY(-M_PI/2, -M_PI/2 , M_PI/2); // return to upright
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose();
    }

    

    void brain() {
      
      //moves to home
      home();
      grip(0);
      geometry_msgs::msg::Pose test;
      test.position.x = 0.48835;
      test.position.y = 0.233;
      test.position.z = 0.17212;
      
      tf2::Quaternion q;
      q.setRPY(-M_PI, 0 , M_PI/2);
      test.orientation.x = q.x();
      test.orientation.y = q.y();
      test.orientation.z = q.z();
      test.orientation.w = q.w();

      pickup(test);
      sleep(2);
      pour(0.1);

     sleep(100.0); 
    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher_;
    geometry_msgs::msg::Pose curr_pose;
    geometry_msgs::msg::Pose old_pose;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<test_brain>());
  rclcpp::shutdown();
  return 0;
}