#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.h"
// #include "navigation_tools/Navigation.h"

enum SMState
{
    SM_INIT,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_DESTINATION,
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
    std::string destination;
    bool finished = false;

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_INIT:
                std::cout << "[SM_INIT]" << std::endl;
                hri->say("Hello, I am Justina");
                hri->say("Please tell me where to navigate");
                state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:
                std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;
                recog = hri->listen("demo.json");
                std::cout << "Recognized: " << recog << std::endl;

                if (!recog.empty())
                {
                    destination = recog;
                    state = SM_NAVIGATE_TO_DESTINATION;
                }
                break;

            case SM_NAVIGATE_TO_DESTINATION:
                std::cout << "[SM_NAVIGATE_TO_DESTINATION]" << std::endl;
                hri->say("I am going to " + destination);

                // goal = nav->getCoordinates(destination);
                // nav->getClose(goal[0], goal[1], goal[2], 180000);

                hri->say("I have arrived at " + destination);
                state = SM_FINAL_STATE;
                break;

            case SM_FINAL_STATE:
                std::cout << "[SM_FINAL_STATE]" << std::endl;
                hri->say("I have finished the navigation demo");
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

