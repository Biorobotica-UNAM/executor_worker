#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "navigation_tools/Navigation.hpp"
//#include "vision_tools/Vision.hpp"

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
    auto manipulation = std::make_shared<Manipulation>(node);
    auto navigation = std::make_shared<Navigation>(node);
    //auto vision = std::make_shared<Vision>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_START_TASK;
    std::string recognized_speech;
    bool person = false;
    bool finished = false;

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_START_TASK:
                std::cout << "[SM_START_TASK]" << std::endl;
                hardware->MoveHead(0.0,0.0);
                manipulation->MoveArmTo("neutral");
                hardware->RelaxGripper();
                hri->Say("My name is Justina. I am ready for help me carry task");
                state = SM_FIND_PERSON;
                break;

            case SM_FIND_PERSON:
                std::cout << "[SM_FIND_PERSON]" << std::endl;
                person = hri->FindPerson();
                if(person == true)
                {
                    hri->Say("Say, Justina follow me, when you are ready");
                    recognized_speech = hri->Listen("help_me_carry.json");
                    std::cout << recognized_speech << std::endl;
                    if(recognized_speech == "justina follow me" || recognized_speech == "robot follow me" || recognized_speech == "follow me")
                    {
                        hri->Say("I am going to follow you, please say, Justina we arrived, once we reach our destination");
                        state = SM_FOLLOW_OPERATOR;
                    }
                }
                else
                {
                    hri->Say("I could not find you, please stand infront of me");
                }
                break;

            case SM_FOLLOW_OPERATOR:
                std::cout << "[SM_FOLLOW_OPERATOR]" << std::endl;
                hri->FollowPerson(true);
                person = hri->FindPerson();
                if(person == false)
                {
                    hri->FollowPerson(false);
                    hri->Say("I lost you, please stand infront of me");
                    state = SM_FIND_PERSON;
                }
                recognized_speech = hri->Listen("help_me_carry.json");
                std::cout << recognized_speech << std::endl;
                if (recognized_speech == "justina we arrived" || recognized_speech == "robot we arrived"  || recognized_speech == "we arrived") 
                {
                    hri->FollowPerson(false);
                    state = SM_CONFIRM_ARRIVAL;
                }
                break;

            case SM_CONFIRM_ARRIVAL:
                std::cout << "[SM_CONFIRM_ARRIVAL]" << std::endl;
                hri->Say("Have we arrived? Answer, Justina yes, or, Justina no");
                recognized_speech = hri->Listen("help_me_carry.json");
                std::cout << recognized_speech << std::endl;
                if(recognized_speech == "justina yes" || recognized_speech == "robot yes" || recognized_speech == "yes")
                {
                    state = SM_FIND_BAG;
                }
                else
                {
                    state = SM_FIND_PERSON;
                } 
                break;

            case SM_FIND_BAG:
                std::cout << "[SM_FIND_BAG]" << std::endl;
                hri->Say("Point at the bag you would like me to carry. Then say, Justina yes");
                recognized_speech = hri->Listen("help_me_carry.json");
                std::cout << recognized_speech << std::endl;
                if(recognized_speech == "justina yes" || recognized_speech == "robot yes" || recognized_speech == "yes")
                {
                    state = SM_CONFIRM_BAG;
                }
                break;

            case SM_CONFIRM_BAG:
                std::cout << "[SM_CONFIRM_BAG]" << std::endl;
                //vision->FindPointingHand();
                hardware->MoveHead(0.0,0.0);
                manipulation->MoveArmTo("pointing_right");
                hri->Say("Are you pointing at that bag? Answer, Justina yes, or, Justina no");
                recognized_speech = hri->Listen("help_me_carry.json");
                std::cout << recognized_speech << std::endl;
                if(recognized_speech == "justina yes" || recognized_speech == "robot yes" || recognized_speech == "yes")
                {
                    state = SM_TAKE_BAG;
                }
                else
                {
                    state = SM_FIND_BAG;
                }
                break;

            case SM_TAKE_BAG:
                std::cout << "[SM_TAKE_BAG]" << std::endl;
                hardware->MoveHead(0.0,0.0);
                manipulation->MoveArmTo("take");
                hardware->OpenGripper();
                hri->Say("Place the bag in my hand. Say, Justina yes, once it is safe to close it");
                recognized_speech = hri->Listen("help_me_carry.json");
                std::cout << recognized_speech << std::endl;
                if(recognized_speech == "justina yes" || recognized_speech == "robot yes" || recognized_speech == "yes")
                {
                    hardware->CloseGripper();
                    state = SM_NAVIGATE_TO_START;
                }
                break;

            case SM_NAVIGATE_TO_START:
                std::cout << "[SM_NAVIGATE_TO_START]" << std::endl;
                hri->Say("I will navigate to my starting point");
                navigation->GoTo("door");
                state = SM_DROP_BAG;
                break;

            case SM_DROP_BAG:
                std::cout << "[SM_DROP_BAG]" << std::endl;
                hardware->MoveHead(0.0,0.0);
                manipulation->MoveArmTo("drop");
                hardware->OpenGripper();
                state = SM_FINISH_TASK;
                break;

            case SM_FINISH_TASK:
                std::cout << "[SM_FINISH_TASK]" << std::endl;
                hri->Say("I have completed help me carry task");
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
