#include <memory>
#include <string>
#include <iostream>

#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"

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

// ====== Variables globales (simple para mantener tu main) ======
sensor_msgs::msg::LaserScan laserScan_;
bool flag_door_ = false;

// ====== Callback ROS2 ======
void callbackLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    laserScan_ = *msg;

    int range = static_cast<int>(laserScan_.ranges.size());
    if (range == 0)
        return;

    int range_c = range / 2;
    int span = std::max(1, range / 10);
    int range_i = std::max(0, range_c - span);
    int range_f = std::min(range - 1, range_c + span);

    int cont_laser = 0;
    float laser_l = 0.0f;

    for (int i = range_i; i <= range_f; i++)
    {
        const float d = laserScan_.ranges[i];
        if (d > 0.0f && d < 4.0f)
        {
            laser_l += d;
            cont_laser++;
        }
    }

    // Evitar división entre cero si no hubo lecturas válidas
    if (cont_laser == 0)
    {
        flag_door_ = false;
        return;
    }

    float prom = laser_l / cont_laser;

    // Tu misma lógica: > 0.50 => puerta abierta
    flag_door_ = (prom > 0.50f);

    // Debug opcional:
    // std::cout << "[Laser] prom=" << prom << " door=" << (flag_door_ ? "OPEN" : "CLOSED") << std::endl;
}


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
                hri->Say("My name is Justina. I am ready for robot inspection task");
                state = SM_NAVIGATE_TO_ENTRANCE;
                break;
            case SM_NAVIGATE_TO_ENTRANCE:
                std::cout << "[SM_NAVIGATE_TO_ENTRANCE]" << std::endl;
                navigation->GoTo("door");
                state = SM_NAVIGATE_TO_ENTRANCE;
                break;

            case SM_WAIT_FOR_DOOR:
                std::cout << "SM_WAIT_FOR_DOOR" << std::endl;
                hri->Say("I am waiting for the door to be open");
                //flag_door = false;
                if(!flag_door_)
                    state = SM_WAIT_FOR_DOOR;
                else
                    state = SM_NAVIGATE_TO_INSPECTION;
                break;


            case SM_NAVIGATE_TO_INSPECTION:
                std::cout << "[SM_NAVIGATE_TO_INSPECTION]" << std::endl;
                hri->Say("I will navigate to the inspection point");
                navigation->GoTo("kitchen");
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
                hri->Say("I will navigate to the exit point");
                navigation->GoTo("door");
                state = SM_FINISH_TASK;
                break;

            case SM_FINISH_TASK:
                std::cout << "[SM_FINISH_TASK]" << std::endl;
                hri->Say("I have completed robot inspection task");
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
