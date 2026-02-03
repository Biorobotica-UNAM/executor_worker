#include <memory>
#include <string>
#include <iostream>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"
#include "navigation_tools/Navigation.hpp"
#include "planning_tools/Planning.hpp"

enum SMState
{
    SM_START_TASK,
    SM_NAVIGATE_TO_ENTRANCE,
    SM_WAIT_FOR_DOOR,
    SM_NAVIGATE_TO_INSPECTION,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_EXIT,
    SM_FINISH_TASK
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_inspection");

    // ===== Tools =====
    auto hri        = std::make_shared<HRI>(node);
    auto hardware   = std::make_shared<Hardware>(node);
    auto navigation = std::make_shared<Navigation>(node);
    auto planning   = std::make_shared<Planning>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_START_TASK;

    std::string recognized_speech;
    std::vector<std::string> planning_result;
    bool finished = false;
    
    // Void say to start the module
    hri->Say("");

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_START_TASK:
            {
                std::cout << "[SM_START_TASK]" << std::endl;

                hri->Say("My name is Justina. I am ready for robot inspection task");

                // planning->startPlanning();
                // planning_result = planning->requestInstruction();

                // if (planning_result.size() >= 3)
                //    std::cout << planning_result[1] << " "
                //              << planning_result[2] << std::endl;

                state = SM_NAVIGATE_TO_ENTRANCE;
                break;
            }

            case SM_NAVIGATE_TO_ENTRANCE:
            {
                std::cout << "[SM_NAVIGATE_TO_ENTRANCE]" << std::endl;

                navigation->GoTo("entrance");

               // planning_result = planning->requestInstruction();
               // if (planning_result.size() >= 3)
               //     std::cout << planning_result[1] << " "
               //               << planning_result[2] << std::endl;

                state = SM_WAIT_FOR_DOOR;
                break;
            }

            case SM_WAIT_FOR_DOOR:
            {
                std::cout << "[SM_WAIT_FOR_DOOR]" << std::endl;

                hri->Say("I am waiting for the door to be open");

                bool door_open = hardware->CheckDoor();

                if (door_open)
                {
                    hri->Say("The door is open");
                    state = SM_NAVIGATE_TO_INSPECTION;
                }
                else
                {
                    std::cout << "Door closed, waiting..." << std::endl;
                }
                break;
            }

            case SM_NAVIGATE_TO_INSPECTION:
            {
                std::cout << "[SM_NAVIGATE_TO_INSPECTION]" << std::endl;

                hri->Say("I will navigate to the inspection point");
                navigation->GoTo("kitchen");

                hri->Say(
                    "I have arrived at the inspection point. "
                    "Please tell me to continue to the exit"
                );

                // planning_result = planning->requestInstruction();
                // if (planning_result.size() >= 3)
                //    std::cout << planning_result[1] << " "
                //              << planning_result[2] << std::endl;

                state = SM_WAIT_FOR_COMMAND;
                break;
            }

            case SM_WAIT_FOR_COMMAND:
            {
                std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;

                recognized_speech = hri->Listen("test.json");
                std::cout << recognized_speech << std::endl;

                if (recognized_speech == "continue")
                {
                    state = SM_NAVIGATE_TO_EXIT;
                }
                break;
            }

            case SM_NAVIGATE_TO_EXIT:
            {
                std::cout << "[SM_NAVIGATE_TO_EXIT]" << std::endl;

                hri->Say("I will navigate to the exit point");
                navigation->GoTo("entrance");

                state = SM_FINISH_TASK;
                break;
            }

            case SM_FINISH_TASK:
            {
                std::cout << "[SM_FINISH_TASK]" << std::endl;
                hri->Say("I have completed robot inspection task");
                finished = true;
                break;
            }

            default:
                break;
        }

        rclcpp::spin_some(node);
        loop.sleep();
    }

    rclcpp::shutdown();
    return 0;
}