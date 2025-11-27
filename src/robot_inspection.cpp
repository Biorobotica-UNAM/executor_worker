#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"

enum SMState
{
    SM_START_TASK,
    SM_NAVIGATE_TO_INSPECTION,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_EXIT,
    SM_FINISH_TASK
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_inspection");

    auto hri = std::make_shared<HRI>(node);
    auto navigation = std::make_shared<Navigation>(node);

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
                hri->Say("I am ready for robot inspection");
                state = SM_NAVIGATE_TO_INSPECTION;
                break;

            case SM_NAVIGATE_TO_INSPECTION:
                std::cout << "[SM_NAVIGATE_TO_INSPECTION]" << std::endl;
                hri->Say("I am going to the inspection point");

                navigation->goTo("living_room");

                hri->Say("I have arrived at the inspection point. Please tell me to continue to the exit");
                state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:
                std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;
                recognized_speech = hri->Listen("test.json");

                std::cout << recognized_speech << std::endl;

                if (recognized_speech == "continue")
                {
                    state = SM_NAVIGATE_TO_EXIT;
                }
                break;

            case SM_NAVIGATE_TO_EXIT:
                std::cout << "[SM_NAVIGATE_TO_EXIT]" << std::endl;
                hri->Say("I am going to the exit point");

                navigation->goTo("door");

                state = SM_FINISH_TASK;
                break;

            case SM_FINISH_TASK:
                std::cout << "[SM_FINISH_TASK]" << std::endl;
                hri->Say("I have finished the test");
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
