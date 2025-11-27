#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"
#include "navigation_tools/Navigation.hpp"
#include "vision_tools/Vision.hpp"

enum SMState
{
    SM_START_TASK,
    SM_FIND_PERSON,
    SM_FOLLOW_OPERATOR,
    SM_CONFIRM_ARRIVAL,
    SM_FIND_BAG,
    SM_CONFIRM_BAG,
    SM_TAKE_BAG,
    SM_NAVIGATE_TO_START,
    SM_DROP_BAG,
    SM_FINISH_TASK
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("help_me_carry");

    auto hri = std::make_shared<HRI>(node);
    auto hardware = std::make_shared<Hardware>(node);
    auto navigation = std::make_shared<Navigation>(node);
    auto vision = std::make_shared<Vision>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_START_TASK;
    std::string recognized_speech;
    bool finished = false;

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_START_TASK:
                std::cout << "[SM_START_TASK]" << std::endl;
                hri->Say("Starting help me carry task");
                state = SM_FIND_PERSON;
                break;

            case SM_FIND_PERSON:
                std::cout << "[SM_FIND_PERSON]" << std::endl;
                hri->Say("Searching for the operator");
                state = SM_FOLLOW_OPERATOR;
                break;

            case SM_FOLLOW_OPERATOR:
                std::cout << "[SM_FOLLOW_OPERATOR]" << std::endl;
                hri->Say("Following the operator");
                state = SM_CONFIRM_ARRIVAL;
                break;

            case SM_CONFIRM_ARRIVAL:
                std::cout << "[SM_CONFIRM_ARRIVAL]" << std::endl;
                hri->Say("I have arrived with the operator");
                state = SM_FIND_BAG;
                break;

            case SM_FIND_BAG:
                std::cout << "[SM_FIND_BAG]" << std::endl;
                hri->Say("Looking for the bag");
                state = SM_CONFIRM_BAG;
                break;

            case SM_CONFIRM_BAG:
                std::cout << "[SM_CONFIRM_BAG]" << std::endl;
                hri->Say("Confirming bag location");
                state = SM_TAKE_BAG;
                break;

            case SM_TAKE_BAG:
                std::cout << "[SM_TAKE_BAG]" << std::endl;
                hri->Say("Taking the bag");
                state = SM_NAVIGATE_TO_START;
                break;

            case SM_NAVIGATE_TO_START:
                std::cout << "[SM_NAVIGATE_TO_START]" << std::endl;
                hri->Say("Returning to the starting point");
                state = SM_DROP_BAG;
                break;

            case SM_DROP_BAG:
                std::cout << "[SM_DROP_BAG]" << std::endl;
                hri->Say("Dropping the bag");
                state = SM_FINISH_TASK;
                break;

            case SM_FINISH_TASK:
                std::cout << "[SM_FINISH_TASK]" << std::endl;
                hri->Say("Help me carry task completed");
                finished = true;
                break;

            default:
                break;
        }

        rclcpp::spin_some(node);
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
