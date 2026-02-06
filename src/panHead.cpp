#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"
#include "vision_tools/Vision.hpp"

enum SMState
{
    SM_START_TASK,
    MOVE_HEAD_LEFT,
    MOVE_HEAD_RIGHT,
    MOVE_HEAD_UP,
    MOVE_HEAD_DOWN,
    MOVE_HEAD_RIGHT_2,
    FOUND,
    NOT_FOUND
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("panHead");

    auto hri = std::make_shared<HRI>(node);
    auto hardware = std::make_shared<Hardware>(node);
    auto vision = std::make_shared<Vision>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_START_TASK;
    bool finished = false;

    // ---------- Límites en grados  ----------
    //const double PAN_LEFT  = 45;
    //const double PAN_ZERO = 0;
    //const double PAN_RIGHT  = -45;

    //const double TILT_UP  = -37;
    //const double TILT_ZERO = 0;
    //const double TILT_DOWN  = 24;
    
    // ---------- Límites en radianes ----------
    const double PAN_LEFT  =  0.785398;   //  45°
    const double PAN_ZERO  =  0.0;        //   0°
    const double PAN_RIGHT = -0.785398;   // -45°

    const double TILT_UP   = -0.645772;   // -37°
    const double TILT_ZERO =  0.0;        //   0°
    const double TILT_DOWN =  0.418879;   //  24°

    double pan = PAN_ZERO;
    double tilt = TILT_ZERO;
    int count = 0;
    int loops = 30;
    int loop_counter = 30;

    while (rclcpp::ok() && !finished)
    {
        // -------- VISIÓN (prioridad máxima) --------
        //if (vision->detectByClass(apple, yolo))
        //{
        //    std::cout << "[OBJECT DETECTED]" << std::endl;
        //    state = FOUND;
        //}

        switch (state)
        {
            case SM_START_TASK:
                std::cout << "[START] pan=" << pan << " tilt=" << tilt << std::endl;
                pan = PAN_ZERO;
                tilt = TILT_ZERO;
                count = 0;
                hardware->MoveHead(pan, tilt);
                state = MOVE_HEAD_RIGHT;
                break;

            case MOVE_HEAD_RIGHT:
                std::cout << "[MOVE_HEAD_RIGHT] pan=" << pan << " tilt=" << tilt << std::endl;
                //delay
                if (loop_counter < loops)
                {
                    loop_counter++;
                    break;
                }
                loop_counter = 0;
                //move head
                pan -= 0.785398;
                if (pan <= -0.785398)
                {
                    state = MOVE_HEAD_DOWN;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_DOWN:
                std::cout << "[MOVE_HEAD_DOWN] pan=" << pan << " tilt=" << tilt << std::endl;
                //delay
                if (loop_counter < loops)
                {
                    loop_counter++;
                    break;
                }
                loop_counter = 0;
                //move head
                tilt = 0.418879;
                state = MOVE_HEAD_LEFT;
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_LEFT:
                std::cout << "[MOVE_HEAD_LEFT] pan=" << pan << " tilt=" << tilt << std::endl;
                //delay
                if (loop_counter < loops)
                {
                    loop_counter++;
                    break;
                }
                loop_counter = 0;
                //move head
                pan += 0.785398;
                if (pan >= 0.785398)
                {
                    state = MOVE_HEAD_UP;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_UP:
                std::cout << "[MOVE_HEAD_UP] pan=" << pan << " tilt=" << tilt << std::endl;
                //delay
                if (loop_counter < loops)
                {
                    loop_counter++;
                    break;
                }
                loop_counter = 0;
                //move head
                tilt = 0;
                state = MOVE_HEAD_RIGHT_2;
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_RIGHT_2:
                std::cout << "[MOVE_HEAD_RIGHT_2] pan=" << pan << " tilt=" << tilt << std::endl;
                //delay
                if (loop_counter < loops)
                {
                    loop_counter++;
                    break;
                }
                loop_counter = 0;
                //ejecuta la rutina 2 veces
                count++;
                pan = 0;
                if (count < 2)
                {
                    state = MOVE_HEAD_RIGHT;   // inicia siguiente rutina
                }
                else
                {
                    state = NOT_FOUND;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case FOUND:
                std::cout << "[FOUND] pan=" << pan << " tilt=" << tilt << std::endl;
                hardware->MoveHead(pan, tilt);
                hri->Say("Person found");
                finished = true;
                break;

            case NOT_FOUND:
                std::cout << "[NOT_FOUND] pan=" << pan << " tilt=" << tilt << std::endl;
                hardware->MoveHead(PAN_ZERO, TILT_ZERO);
                hri->Say("No person found");
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