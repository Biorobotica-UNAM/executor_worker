#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
// #include "navigation_tools/Navigation.h"

enum SMState
{
    SM_INIT,
    SM_NAVIGATE_TO_INSPECTION,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_EXIT,
    SM_FINAL_STATE
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_inspection");

    auto hri = std::make_shared<HRI>(node);
    // auto nav = std::make_shared<Navigation>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_INIT;
    std::vector<float> goal(3);
    std::string recog;
    bool finished = false;

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_INIT:
                std::cout << "[SM_INIT]" << std::endl;
                hri->Say("I am ready for robot inspection");
                state = SM_NAVIGATE_TO_INSPECTION;
                break;

            case SM_NAVIGATE_TO_INSPECTION:
                std::cout << "[SM_NAVIGATE_TO_INSPECTION]" << std::endl;
                hri->Say("I am going to the inspection point");

                // goal = nav->getCoordinates("entrance_door");
                // nav->getClose(goal[0], goal[1], goal[2], 120000);

                hri->Say("I have arrived at the inspection point");
                hri->Say("Please tell me to continue to the exit");
                state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:
                std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;
                recog = hri->listen("test.json");

                std::cout << recog << std::endl;

                if (recog == "continue")
                {
                    state = SM_NAVIGATE_TO_EXIT;
                }
                break;

            case SM_NAVIGATE_TO_EXIT:
                std::cout << "[SM_NAVIGATE_TO_EXIT]" << std::endl;
                hri->Say("I am going to the exit point");

                // goal = nav->getCoordinates("inspection_point");
                // nav->getClose(goal[0], goal[1], goal[2], 180000);

                state = SM_FINAL_STATE;
                break;

            case SM_FINAL_STATE:
                std::cout << "[SM_FINAL_STATE]" << std::endl;
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
