#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>
#include <string>
#include <functional>
#include "brain_msgs/msg/command.hpp"
#include <vector>


using std::placeholders::_1;

class main_brain : public rclcpp::Node {
    

    public:
    main_brain() : Node("main_brain") {

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        publisher_ = this->create_publisher<brain_msgs::msg::Command>("command",10);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
      "keyboard_input", 10, std::bind(&main_brain::executeCommand, this, _1));
      

        for (auto& checkDrink : drinkOptions) {
        std::cout << "NEXT DRINK" << std::endl;
        std::cout << checkDrink.name << std::endl;
            for (auto& ing : checkDrink.ingredients) {
                std::cout << ing << std::endl;
            }
        }
        //timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&main_brain::tfCallback, this));

    };

    private:

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

    void executeCommand(const std_msgs::msg::String::SharedPtr msg) const {
        std::string drink;
        brain_msgs::msg::Command instruction;
        if (msg->data == "space") {
            std::cout << "input: ";
            std::cin >> drink;
            std::cout << drink << std::endl;
        }

        instruction.command.data = "pickup";
        instruction.item.data = drink;
        for (auto& checkDrink : drinkOptions) {
            if (checkDrink.name == drink) {
                std::cout << "Drink found!" << std::endl;
                for (auto& ing : checkDrink.ingredients) {
                    std::cout << ing << std::endl;
                }
                break;
            }
        }

        instruction.item_pose = t; 
        tfCallback(req_frame);
        publisher_->publish(instruction);

    }
    struct drinkTemplate {
        std::string name;
        std::vector<std::string> ingredients;
    };

    rclcpp::Publisher<brain_msgs::msg::Command>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped t;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::string req_frame;

    std::vector<struct drinkTemplate> drinkOptions = {
        drinkTemplate{"shot",{"vodka"}},
        drinkTemplate{"vodka cranberry",{"vodka","c.juice"}},
        drinkTemplate{"margarita",{"tequila","lim.juice","lem.juice"}}
    };
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<main_brain>());
  rclcpp::shutdown();
  return 0;
}