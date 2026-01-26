#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"
#include "planning_tools/Planning.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

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
    auto node = rclcpp::Node::make_shared("demo_node");

    auto hri = std::make_shared<HRI>(node);
    auto navigation = std::make_shared<Navigation>(node);
    auto planning = std::make_shared<Planning>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_INIT;
    std::string destination;
    std::vector<std::string> planning_result;
    bool finished = false;

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_INIT:
                std::cout << "[SM_INIT]" << std::endl;

                hri->Say("Hello, I am Justina");
                hri->Say("Please tell me where to navigate");                
                                    
                state = SM_WAIT_FOR_COMMAND;
                break;

            case SM_WAIT_FOR_COMMAND:
                std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;

                planning->interpretToClips("Ve a la cocina");
                planning_result = planning->requestInstruction();                    
                destination = planning_result[2];

                state = SM_NAVIGATE_TO_DESTINATION;
                break;

            case SM_NAVIGATE_TO_DESTINATION:
                std::cout << "[SM_NAVIGATE_TO_DESTINATION]" << std::endl;

                hri->Say("I am going to " + destination);

                navigation->GoTo(destination);

                hri->Say("I have arrived at " + destination);

                state = SM_FINAL_STATE;
                break;

            case SM_FINAL_STATE:
                std::cout << "[SM_FINAL_STATE]" << std::endl;

                hri->Say("I have finished the navigation demo");
                
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
