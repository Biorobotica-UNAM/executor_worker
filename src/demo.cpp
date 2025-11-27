#include <chrono>
#include <memory>
#include <vector>
#include <string>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "service/srv/string.hpp"  

enum SMState
{
    SM_INIT,
    SM_WAIT_FOR_COMMAND,
    SM_NAVIGATE_TO_DESTINATION,
    SM_FINAL_STATE
};

std::string last_instruction_third_token = "";

// Función para extraer el último token (zona) del mensaje
std::string extract_zone_from_message(const std::string& message)
{
    std::stringstream ss(message);
    std::string token;
    std::string last_token;
    
    // Extraer todos los tokens y quedarnos con el último
    while (ss >> token)
    {
        last_token = token;
    }
    
    return last_token;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("robot_inspection");

    auto hri = std::make_shared<HRI>(node);
    auto nav = std::make_shared<Navigation>(node);

    // --- Cliente para el servicio /request_instruction ---
    auto service_client = node->create_client<std_srvs::srv::Trigger>("/request_instruction");
    
    // --- Cliente para el servicio natural_to_clips/interpret ---
    auto interpret_client = node->create_client<service::srv::String>("/natural_to_clips/interpret");
    
    // Esperar a que ambos servicios estén disponibles
    while (!service_client->wait_for_service(std::chrono::seconds(1)) || 
           !interpret_client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for services.");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "Services not available, waiting again...");
    }

    rclcpp::Rate loop(10);
    SMState state = SM_INIT;
    std::vector<float> goal(3);
    std::string recog;
    std::string destination;
    bool finished = false;
    bool service_called = false;
    bool interpret_service_called = false;  // Bandera para el servicio de interpretación

    while (rclcpp::ok() && !finished)
    {
        switch (state)
        {
            case SM_INIT:
                {
                    std::cout << "[SM_INIT]" << std::endl;
                    hri->Say("Hello, I am Justina");
                    hri->Say("Please tell me where to navigate");
                    
                    // Llamar al servicio de interpretación
                    if (!interpret_service_called) {
                        std::cout << "Calling service natural_to_clips/interpret..." << std::endl;
                        
                        auto interpret_request = std::make_shared<service::srv::String::Request>();
                        // Configurar el mensaje de solicitud si es necesario
                        interpret_request->data = "Ve a la cocina";
                        
                        auto interpret_future = interpret_client->async_send_request(interpret_request);
                        
                        // Esperar la respuesta con timeout
                        if (rclcpp::spin_until_future_complete(node, interpret_future, std::chrono::seconds(5)) 
                            == rclcpp::FutureReturnCode::SUCCESS) 
                        {
                            auto interpret_response = interpret_future.get();
                            std::cout << "Interpret service returned: " << interpret_response->data << std::endl;
                            // Procesar la respuesta
                        } else {
                            std::cout << "Interpret service call failed or timed out" << std::endl;
                        }
                        
                        interpret_service_called = true;
                    }
                    
                    state = SM_WAIT_FOR_COMMAND;
                }
                break;

            case SM_WAIT_FOR_COMMAND:
                {
                    std::cout << "[SM_WAIT_FOR_COMMAND]" << std::endl;
                    
                    if (!service_called) {
                        std::cout << "Calling service /request_instruction..." << std::endl;

                        // Crear la solicitud del servicio
                        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
                        
                        // Llamar al servicio de forma asíncrona
                        auto future_result = service_client->async_send_request(request);
                        
                        // Esperar la respuesta con un timeout
                        if (rclcpp::spin_until_future_complete(node, future_result, std::chrono::seconds(5)) 
                            == rclcpp::FutureReturnCode::SUCCESS) 
                        {
                            auto response = future_result.get();
                            
                            if (response->success) {
                                std::cout << "Service returned: " << response->message << std::endl;
                                
                                // Extraer solo la zona (último token) del mensaje
                                destination = extract_zone_from_message(response->message);
                                std::cout << "Extracted destination: " << destination << std::endl;
                                
                                state = SM_NAVIGATE_TO_DESTINATION;
                            } else {
                                std::cout << "Service call succeeded but returned failure: " 
                                          << response->message << std::endl;
                                hri->Say("I received an invalid instruction, please try again");
                                // Reset the flag to try again
                                service_called = false;
                            }
                        } else {
                            std::cout << "Service call failed or timed out" << std::endl;
                            hri->Say("I could not get instructions, please try again");
                            // Reset the flag to try again
                            service_called = false;
                        }
                        
                        service_called = true;
                    }
                }
                break;

            case SM_NAVIGATE_TO_DESTINATION:
                std::cout << "[SM_NAVIGATE_TO_DESTINATION]" << std::endl;
                hri->Say("I am going to " + destination);

                nav->GoTo(destination);

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
