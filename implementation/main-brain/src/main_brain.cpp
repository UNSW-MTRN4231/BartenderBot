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
        std::vector<std::string> ingredientNames; 
        std::vector<std::string> ingredientFrames;
        
        if (ingredientIndex < numIngredients) {
            std::cout << armReady;
            ingredientNames.push_back(drink.ingredients.at(ingredientIndex));
            ingredientFrames.push_back(drink.ingredients.at(ingredientIndex));
            std::cout << ingredientNames.at(0) << ", " << ingredientFrames.at(0) << std::endl;
            sendCommand(instruction, ingredientNames, ingredientFrames);
            ingredientIndex++;
        } else {
            state = 2;
        }
        
    }

    void shakeDrink() {
        std::cout << "shaking" << std::endl;
        std::string instruction = "shake";
        std::vector<std::string> ingredientNames; 
        std::vector<std::string> ingredientFrames;
        sendCommand(instruction,ingredientNames,ingredientFrames);
    };

    void serveDrink() {
        std::cout << "serving" << std::endl;
        std::cout << "shaking" << std::endl;
        std::string instruction = "serve";
        std::vector<std::string> ingredientNames = {"glass1","glass2","glass3"}; 
        std::vector<std::string> ingredientFrames = {"glass1","glass2","glass3"};
        sendCommand(instruction,ingredientNames,ingredientFrames);
    }

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


    }
    
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<main_brain>());
  rclcpp::shutdown();
  return 0;
}