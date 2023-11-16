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

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class arm_brain : public rclcpp::Node {
  public:
    arm_brain() : Node("arm_brain")
    {
      parallel_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
      rclcpp::SubscriptionOptions options;
      options.callback_group = parallel_callback_group_;

      // Initalise the brain subscriber
      subscription_ = this->create_subscription<brain_msgs::msg::Command>("command", 10, std::bind(&arm_brain::brain, this, _1), options);

      // Initialise the pose publisher
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("move_to", 10);

      // Initialise the brain ready publisher
      ready_publisher_ = this->create_publisher<std_msgs::msg::String>("ready", 10);

      // Initialise the arduino publisher
      arduino_publisher_ = this->create_publisher<std_msgs::msg::String>("arduinoCommand", 10);

      // Initialise the arm ready publisher
      arm_ready_publisher_ = this->create_publisher<std_msgs::msg::String>("done_move", 10);
      arm_ready_subscriber_ = this->create_subscription<std_msgs::msg::String>("done_move", 10, std::bind(&arm_brain::progressChange, this, _1), options);


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
    //change subscription
    void progressChange(const std_msgs::msg::String &msg) {
      RCLCPP_INFO(this->get_logger(), "recieved");
      if (msg.data == "done") {
        RCLCPP_INFO(this->get_logger(), "Ready for next move");
        ready = 1;
      } else {
        RCLCPP_INFO(this->get_logger(), "Not ready");
        ready = 0;
      }
    }

    //Function to generate a target position message
    geometry_msgs::msg::Pose generatePoseFromTransform(geometry_msgs::msg::TransformStamped tf) {
      geometry_msgs::msg::Pose msg;
    
      msg.position.x = tf.transform.translation.x;
      msg.position.y = tf.transform.translation.y;
      msg.position.z = tf.transform.translation.z;
      RCLCPP_INFO(this->get_logger(), "Position Heard: '%f' '%f' '%f' ", msg.position.x, msg.position.y, msg.position.z);
      // TODO: CHECK ROTATION OF GRIPPER FOR PICKUPS
      
      tf2::Quaternion q;
      q.setRPY(-M_PI, 0 , M_PI/2);
      msg.orientation.x = q.x();
      msg.orientation.y = q.y();
      msg.orientation.z = q.z();
      msg.orientation.w = q.w();
      
      return msg;
    }

    //requests transform frame for object
    geometry_msgs::msg::TransformStamped tfCallback(std::string req_frame) {
      // Check if the transformation is between "world" and "req_frame"
      std::string fromFrameRel = "base_link";
      std::string toFrameRel = req_frame;
      geometry_msgs::msg::TransformStamped t;

      try {
          t = tf_buffer_->lookupTransform(fromFrameRel, toFrameRel, tf2::TimePointZero);
      } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
          
      }
      return t;
    }

    //gets pose of object
    geometry_msgs::msg::Pose get_pose(std::string item_frame) {
        return generatePoseFromTransform(tfCallback(item_frame));
    }
    
    //sends pose to ur
    void send_pose(std::string type) {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose = curr_pose;
      msg.header.frame_id = type;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f' '%f' '%f' ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Publishing orien: '%f' '%f' '%f' '%f' ", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
      pose_publisher_->publish(msg);

      std_msgs::msg::String status;
      status.data = "waiting";
      arm_ready_publisher_->publish(status);
      sleep(4);
      while(!ready) {sleep(2);}
    }

    void send_pose() {
      geometry_msgs::msg::PoseStamped msg;
      msg.pose = curr_pose;
      msg.header.frame_id = "free";
      RCLCPP_INFO(this->get_logger(), "Publishing pos: '%f' '%f' '%f' ", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
      RCLCPP_INFO(this->get_logger(), "Publishing orien: '%f' '%f' '%f' '%f' ", msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
      pose_publisher_->publish(msg);
      
      std_msgs::msg::String status;
      status.data = "waiting";
      arm_ready_publisher_->publish(status);
      sleep(4);
      while(!ready) {sleep(2);}
    }
    
    // toggle gripper
    void grip(bool toggle) {
      std_msgs::msg::String grip;
      if (toggle == 0) {
        grip.data  = "open";
        toggle = 1;
        RCLCPP_INFO(this->get_logger(), "Claw Open");
      }
      else if(toggle == 1) {
        grip.data = "close";
        toggle = 0;
        RCLCPP_INFO(this->get_logger(), "Claw Close");
      }
      arduino_publisher_->publish(grip);
    }

    //returns robot to home position
    void home() {
      curr_pose.position.x = 0.588457;
      curr_pose.position.y = 0.133329;
      curr_pose.position.z = 0.371829;
      
      curr_pose.orientation.x = 0.707;
      curr_pose.orientation.y = 0;
      curr_pose.orientation.z = 0.707;
      curr_pose.orientation.w = 0;
      send_pose("home");
      RCLCPP_INFO(this->get_logger(), "Moving to home position");
    }

    void move(std::string item) {
      
      if (item == "return") {
        curr_pose = old_pose;
      }
      else {
        curr_pose = get_pose(item);
        curr_pose.position.z = 0.37;
      }
      
      send_pose("linear");

      //TODO: CHECK IF OBJECT IS STILL IN LOCATION -> MOVE AGAIN
    }

    // moves to and picks up object
    void pickup(geometry_msgs::msg::Pose pose) {
      tf2::Quaternion q;
      RCLCPP_INFO(this->get_logger(), "Starting pickup");
      if (pose.position.y > 0.3) {
        curr_pose.position.y = pose.position.y - claw.y;
        q.setRPY(M_PI/2 ,-M_PI/2 , M_PI);
      } else {
        curr_pose.position.y = pose.position.y + claw.y;
        q.setRPY(M_PI, -M_PI/2 , -M_PI/2);
      }
      curr_pose.position.x = pose.position.x;
      curr_pose.position.z = pose.position.z + claw.z + 0.15;
      
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();

      send_pose("add");
      grip(0);
      
      curr_pose.position.z = pose.position.z + claw.z;
      send_pose("add");
      send_pose("go");

      grip(1);

      curr_pose.position.z = 0.37;
      send_pose("linear");
    }

    // combines both parts of shaker and mixes, then dissasembles
    void shake(geometry_msgs::msg::Pose big, geometry_msgs::msg::Pose small) {
      // picks up small shaker, moves above other, rotates and combines
      
      pickup(small);
      tf2::Quaternion q;
      //rotates bottle
      if (big.position.y > 0.3) {
        curr_pose.orientation.x = -0.5;
        curr_pose.orientation.y = 0.5;
        curr_pose.orientation.z = 0.5;
        curr_pose.orientation.w = 0.5;
      } else {
        curr_pose.orientation.x = 0.5;
        curr_pose.orientation.y = 0.5;
        curr_pose.orientation.z = -0.5;
        curr_pose.orientation.w = 0.5;
      }
      send_pose("add");

      curr_pose.position.x = big.position.x;
      if (big.position.y > 0.3) {
        curr_pose.position.x = big.position.x - claw.y;
      } else {
        curr_pose.position.x = big.position.x + claw.y;
      }
      send_pose("add");
      curr_pose.position.z -= 0.15;
      send_pose("add");
      send_pose("go");
      grip(0);
      // picks up combined, moves above and rotates
      curr_pose.position.z -= 0.05;
      send_pose("linear");
      grip(1);

      curr_pose.position.z += 0.3;
      send_pose("add");

      //shaking
      if (curr_pose.position.y > 0.3) {
        q.setRPY(M_PI/2 ,-M_PI/2 , M_PI);
        curr_pose.orientation.x = q.x();
        curr_pose.orientation.y = q.y();
        curr_pose.orientation.z = q.z();
        curr_pose.orientation.w = q.w();
        send_pose("add");

        curr_pose.orientation.x = -0.5;
        curr_pose.orientation.y = 0.5;
        curr_pose.orientation.z = 0.5;
        curr_pose.orientation.w = 0.5;
        send_pose("add");
      } else {
        q.setRPY(M_PI, -M_PI/2 , -M_PI/2);
        curr_pose.orientation.x = q.x();
        curr_pose.orientation.y = q.y();
        curr_pose.orientation.z = q.z();
        curr_pose.orientation.w = q.w();
        send_pose("add");

        curr_pose.orientation.x = 0.5;
        curr_pose.orientation.y = 0.5;
        curr_pose.orientation.z = -0.5;
        curr_pose.orientation.w = 0.5;
        send_pose("add");
      }

      // places back down and dissasembles
      curr_pose.position.z -= 0.3;
      send_pose("add");
      send_pose("go");
      grip(0);

      curr_pose.position.z += 0.05;
      send_pose("linear");
      grip(1);

      curr_pose.position.z += 0.10;
      send_pose("add");

      //flips back upright
      if (curr_pose.position.y > 0.3) {
        q.setRPY(M_PI/2 ,-M_PI/2 , M_PI);
      } else {
        q.setRPY(M_PI, -M_PI/2 , -M_PI/2);
      }
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose("add");

      curr_pose.position.x = small.position.x;
      if (big.position.y > 0.3) {
        curr_pose.position.x = small.position.y - claw.y;
      } else {
        curr_pose.position.x = small.position.y + claw.y;
      }
      send_pose("add");
      curr_pose.position.z = small.position.z + claw.z;
      send_pose("add");
      send_pose("go");

      grip(0);
    }
    
    // pours the bottle at current location
    void pour(float offset) {
      offset = (0.07-offset/2)/sqrt(2);
        
      tf2::Quaternion q;
      q.setRPY(0, -M_PI/2, -M_PI); // facing computer
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose("add");
      RCLCPP_INFO(this->get_logger(), "Moving to pour angle");
      if (curr_pose.position.y > 0.3) {
        // moves accross offset width
        curr_pose.position.y = curr_pose.position.y - offset;
        send_pose("add");
        curr_pose.orientation.x = 0.182746;
        curr_pose.orientation.y = 0.683087;
        curr_pose.orientation.z = 0.183275;
        curr_pose.orientation.w = 0.68294; // pour towards
      } else {
        // moves accross offset width
        curr_pose.position.y = curr_pose.position.y + offset;
        send_pose("add");
        curr_pose.orientation.x = -0.183292;
        curr_pose.orientation.y = 0.682972;
        curr_pose.orientation.z = -0.182745;
        curr_pose.orientation.w = 0.68305; // pour away
      }
      // pours drink
      //curr_pose.orientation.w = M_PI/4;
      send_pose("add");
      RCLCPP_INFO(this->get_logger(), "Pouring...");
      sleep(2);
      

      q.setRPY(0, -M_PI/2, -M_PI); // return to upright
      curr_pose.orientation.x = q.x();
      curr_pose.orientation.y = q.y();
      curr_pose.orientation.z = q.z();
      curr_pose.orientation.w = q.w();
      send_pose("add");
      send_pose("go");
    }

    
    void brain(const brain_msgs::msg::Command &msg) {
      
      //moves to home
      home();
      grip(0);
      // check for what command is
      
      // adding ingredient
      
      
      if (msg.command == "fill") {
        std::cout << "fill dunphy" << std::endl;
        //need repeat
        for (unsigned long int i = 0; i < size(msg.item_frames); i++) {
          std::cout << "filling " << i << std::endl;
          old_pose = get_pose(msg.item_frames[i]);
          std::cout << "pose got" << std::endl;
          pickup(old_pose); // to bottle
          // move("big_shaker"); //to shaker
          // pour(msg.item_heights[i]);
          // move("return"); //to old bottle;
          // grip(0); //release
        }
      }
      /*
      // mixing ingredients
      else if (msg.command == "shake") {
        shake(get_pose("big_shaker"), get_pose(msg.item_frames[0]));
      }
      
      // pouring cocktail
      else if (msg.command == "pour") {
        
        old_pose = get_pose("big_shaker");
        pickup(old_pose); // to bottle
          
        //need repeat
        for (unsigned long int i = 0; i < size(msg.item_frames); i++) {
          move(msg.item_frames[i]);
          pour(msg.item_heights[i]);
        }

        move("return"); //to old shaker
        grip(0); //release
      }
      else {
        RCLCPP_INFO( this->get_logger(), "Unknown Command");
      }
      std_msgs::msg::String ready;
      ready.data = "ready";
      ready_publisher_->publish(ready);


      */

    }

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::CallbackGroup::SharedPtr parallel_callback_group_;

    rclcpp::Subscription<brain_msgs::msg::Command>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ready_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arduino_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_ready_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arm_ready_subscriber_;
    geometry_msgs::msg::Pose curr_pose;
    geometry_msgs::msg::Pose old_pose;
    struct offset {
      double y = 0.20;
      double z = 0.1;
    } claw;
    bool ready = 0;

    size_t count_;
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<arm_brain>();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}