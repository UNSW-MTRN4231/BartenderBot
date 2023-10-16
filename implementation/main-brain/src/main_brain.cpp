#include <memory>
#include <chrono>
#include <string>
#include <functional>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "brain_msgs/msg/command.hpp"

using std::placeholders::_1;

class main_brain : public rclcpp::Node {
    

    public:
    main_brain() : Node("main_brain") {

        publisher_ = this->create_publisher<brain_msgs::msg::Command>("command",10);

        subscription_ = this->create_subscription<std_msgs::msg::String>(
        "keyboard_input", 10, std::bind(&main_brain::executeCommand, this, _1));

        continue_sub_ = this->create_subscription<std_msgs::msg::String>(
        "ready", 10, std::bind(&main_brain::checkArm, this, _1));

        for (auto& checkDrink : drinkOptions) {
        std::cout << "NEXT DRINK" << std::endl;
        std::cout << checkDrink.name << std::endl;
            for (auto& ing : checkDrink.ingredients) {
                std::cout << ing << std::endl;
            }
        }
    };

    private:

    // Member Variables
    struct drinkTemplate {
        std::string name;
        std::vector<std::string> ingredients;
    };

    rclcpp::Publisher<brain_msgs::msg::Command>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr continue_sub_;

    bool armReady = true;

    std::vector<struct drinkTemplate> drinkOptions = {
        drinkTemplate{"shot",{"vodka"}},
        drinkTemplate{"vodka cranberry",{"vodka","c.juice"}},
        drinkTemplate{"margarita",{"tequila","lim.juice","lem.juice"}}
    };

    void checkArm(const std_msgs::msg::String::SharedPtr read) {
        armReady = !armReady;
    }

    void sendCommand(std_msgs::msg::String command, std_msgs::msg::String items[], std_msgs::msg::String frames[]) {
        brain_msgs::msg::Command instruction;
        armReady = !armReady;
        instruction.command = command;
        for (size_t i; i < sizeof(items)/sizeof(std_msgs::msg::String); i++) {
            std::cout << "Ingredient" << std::endl;
            instruction.item_names[i] = items[i];
            instruction.item_frames[i] = frames[i];
        }
        
        publisher_->publish(instruction);
    }

    void fillShaker(struct drinkTemplate& drink) {
        int drinkNum = 0;
        int numIngredients = drink.ingredients.size();
        std_msgs::msg::String instruction;
        std_msgs::msg::String ingredientName[1]; 
        std_msgs::msg::String ingredientFrame[1];
        instruction.data = "fill";
        std::cout << armReady;
        while (drinkNum <= numIngredients) {
            if (armReady) {
                ingredientName[0].data = {drink.ingredients.at(drinkNum)};
                ingredientFrame[0].data = {drink.ingredients.at(drinkNum)};
                sendCommand(instruction, ingredientName, ingredientFrame);
                drinkNum++;
            }
        }
    }

    // void shakeDrink() {
    //     std::cout << "shaking" << std::endl;
    // };

    // void serveDrink() {
    //     std::cout << "serving" << std::endl;
    // }

    void executeCommand(const std_msgs::msg::String::SharedPtr msg) {
        std::string drink;
        
        
        if (msg->data == "space") {
            std::cout << "input: ";
            std::cin >> drink;
            std::cout << drink << std::endl;
        }
        struct drinkTemplate chosen;
        for (auto& checkDrink : drinkOptions) {
            if (checkDrink.name == drink) {
                std::cout << "Drink found!" << std::endl;
                chosen = checkDrink;
                for (auto& ing : checkDrink.ingredients) {
                    std::cout << ing << std::endl;
                }
                break;
            }
        }
        
        std::cout << "BEGIN FILL" << std::endl;
        fillShaker(chosen);
        std::cout << "END FILL" << std::endl;

        // std::cout << "BEGIN SHAKE" << std::endl;
        // shakeDrink();
        // std::cout << "END SHAKE" << std::endl;

        // std::cout << "BEGIN SERVE" << std::endl;
        // serveDrink();
        // std::cout << "END SERVE" << std::endl;
        
    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<main_brain>());
  rclcpp::shutdown();
  return 0;
}