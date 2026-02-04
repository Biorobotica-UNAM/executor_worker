#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"
//#include "vision_tools/Vision.hpp"

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
    //auto vision = std::make_shared<Vision>(node);

    rclcpp::Rate loop(10);
    SMState state = SM_START_TASK;
    bool finished = false;

    // ---------- Límites en bits ----------
    const double PAN_MIN  = 1024;
    const double PAN_ZERO = 2057;
    const double PAN_MAX  = 3091;

    const double TILT_MIN  = 3100;
    const double TILT_ZERO = 3526;
    const double TILT_MAX  = 3794;

    // ---------- Límites en grados (referencia) ----------
    const double PAN_MIN_DEG  = 90;
    const double PAN_ZERO_DEG = 0;
    const double PAN_MAX_DEG  = -90;

    const double TILT_MIN_DEG  = -37;
    const double TILT_ZERO_DEG = 0;
    const double TILT_MAX_DEG  = 24;
    
    //const double deg_p_bit = 

    const double STEP = 3;   // bits por iteración. Modificar para igualar la velocidad real de los motores

    double pan = PAN_ZERO;
    double tilt = TILT_ZERO;
    int count = 0;

    while (rclcpp::ok() && !finished)
    {
        // -------- VISIÓN (prioridad máxima) --------
        //if (vision->PersonDetected())
        //{
        //    std::cout << "[PERSON DETECTED]" << std::endl;
        //    state = FOUND;
        //}

        switch (state)
        {
            case SM_START_TASK:
                std::cout << "[START]" << std::endl;
                pan = PAN_ZERO;
                tilt = TILT_ZERO;
                count = 0;
                hardware->MoveHead(pan, tilt);
                state = MOVE_HEAD_RIGHT;
                break;

            case MOVE_HEAD_RIGHT:
                std::cout << "[MOVE_HEAD_RIGHT]" << std::endl;

                pan -= STEP;
                if (pan <= PAN_MIN)
                {
                    pan = PAN_MIN;
                    state = MOVE_HEAD_DOWN;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_DOWN:
                std::cout << "[MOVE_HEAD_DOWN]" << std::endl;

                tilt += STEP;
                if (tilt >= TILT_MAX)
                {
                    tilt = TILT_MAX;
                    state = MOVE_HEAD_LEFT;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_LEFT:
                std::cout << "[MOVE_HEAD_LEFT]" << std::endl;

                pan += STEP;
                if (pan >= PAN_MAX)
                {
                    pan = PAN_MAX;
                    state = MOVE_HEAD_UP;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_UP:
                std::cout << "[MOVE_HEAD_UP]" << std::endl;

                tilt -= STEP;
                if (tilt <= TILT_MIN)
                {
                    tilt = TILT_MIN;
                    state = MOVE_HEAD_RIGHT_2;
                }
                hardware->MoveHead(pan, tilt);
                break;

            case MOVE_HEAD_RIGHT_2:
                std::cout << "[MOVE_HEAD_RIGHT_2]" << std::endl;

                pan -= STEP;

                if (pan <= PAN_ZERO)   // tolerancia
                {
                    pan = PAN_ZERO;
                    count++;

                    if (count < 2)
                    {
                        state = MOVE_HEAD_RIGHT;   // inicia siguiente rutina
                    }
                    else
                    {
                        state = NOT_FOUND;
                    }
                }
                hardware->MoveHead(pan, tilt);
                break;

            case FOUND:
                std::cout << "[FOUND]" << std::endl;
                hardware->MoveHead(pan, tilt);
                hri->Say("Person found");
                finished = true;
                break;

            case NOT_FOUND:
                std::cout << "[NOT_FOUND]" << std::endl;
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