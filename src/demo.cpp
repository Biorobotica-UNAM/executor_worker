#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.h"
#include "navigation_tools/Navigation.h"
#include "std_msgs/msg/string.hpp"

enum SMState
{
    SM_INIT,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_DESTINATION,
    SM_FINAL_STATE
};

std::string last_instruction_third_token = "";

void instruction_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::stringstream ss(msg->data);
    std::string token;
    int count = 0;
    while (ss >> token)
    {
        count++;
        if (count == 3)
        {
            last_instruction_third_token = token;
            break;
        }
    }
    std::cout << "[Subscriber] Received: " << msg->data 
              << " | Third token: " << last_instruction_third_token << std::endl;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_inspection");

    auto hri = std::make_shared<HRI>(node);
    auto nav = std::make_shared<Navigation>(node);

    // --- Nuevo subscriber ---
    auto instruction_sub = node->create_subscription<std_msgs::msg::String>(
        "/instruction_msg",
        10,
        instruction_callback
    );
    // -------------------------

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
                std::cout << "Waiting for message on /instruction_msg..." << std::endl;

                // Usa el token recibido como destino
                if (!last_instruction_third_token.empty())
                {
                    destination = last_instruction_third_token;
                    std::cout << "Destination: " << destination << std::endl;
                    state = SM_NAVIGATE_TO_DESTINATION;
                }
                break;

            case SM_NAVIGATE_TO_DESTINATION:
                std::cout << "[SM_NAVIGATE_TO_DESTINATION]" << std::endl;
                hri->say("I am going to " + destination);

                nav->goTo(destination);

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
