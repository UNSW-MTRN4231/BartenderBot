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
        "ready", 10, std::bind(&main_brain::executeCommand, this, _1));

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

    int state = -1;

    bool armReady = true;
    struct drinkTemplate chosenDrink;
    int ingredientIndex = 0;

    std::vector<struct drinkTemplate> drinkOptions = {
        drinkTemplate{"shot",{"vodka"}},
        drinkTemplate{"vodka cranberry",{"vodka","c.juice"}},
        drinkTemplate{"margarita",{"tequila","lim.juice","lem.juice"}}
    };

    void sendCommand(std::string command, std::vector<std::string> items, std::vector<std::string> frames) {
        brain_msgs::msg::Command instruction;
        instruction.command.data = command;
        for (size_t i=0; i < items.size();i++) {
            std::cout << "Ingredient" << std::endl;
            instruction.item_names[i].data = items.at(i);
            instruction.item_frames[i].data = frames.at(i);
        }
        std::cout << instruction.item_names[0].data << ", " << instruction.item_frames[0].data << std::endl;
        
        publisher_->publish(instruction);
        std::cout << "Ingredient" << std::endl;
    }

    void fillShaker(struct drinkTemplate& drink) {
        int numIngredients = drink.ingredients.size();
        std::string instruction = "fill";
        std::vector<std::string> ingredientName; 
        std::vector<std::string> ingredientFrame;
        
        if (ingredientIndex < numIngredients) {
            std::cout << armReady;
            ingredientName.push_back(drink.ingredients.at(ingredientIndex));
            ingredientFrame.push_back(drink.ingredients.at(ingredientIndex));
            std::cout << ingredientName.at(0) << ", " << ingredientFrame.at(0) << std::endl;
            sendCommand(instruction, ingredientName, ingredientFrame);
            ingredientIndex++;
        } else {
            state = 2;
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
        switch (state) {
            
            case -1:
                
                if (msg->data == "space") {
                    std::cout << "input: ";
                    std::cin >> drink;
                    std::cout << drink << std::endl;
                    
                    for (auto& checkDrink : drinkOptions) {
                        if (checkDrink.name == drink) {
                            std::cout << "Drink found!" << std::endl;
                            chosenDrink = checkDrink;
                            for (auto& ing : checkDrink.ingredients) {
                                std::cout << ing << std::endl;
                            }
                            break;
                        }
                    }
                    state = 1;
                    fillShaker(chosenDrink);
                }
                break;
            case 1:
                fillShaker(chosenDrink);
                break;
            case 2:
                std::cout << "SHAKE" << std::endl;
                state = 3;
                break;
            case 3:
                std::cout << "SERVE" << std::endl;
                state = 5;
                break;
            case 5:
                std::cout << "DRINK DONE" << std::endl;
                state = -1;
                break;

        }

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