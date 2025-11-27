#include <iostream>
#include <sstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/logger.hpp>
#include <rclcpp/time.hpp>

#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <tuple>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "robotino_msgs/DigitalReadings.h"
#include "actionlib_msgs/GoalStatus.h"


//Festino Tools
#include "festino_tools/HRI.hpp"
//#include "festino_tools/Vision.h"
#include "festino_tools/Navigation.hpp"
//#include "festino_tools/Hardware.hpp"

//Digital readings
#include "robotino_msgs/DigitalReadings.h"

//TF
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/pose_stamped.hpp"


//Nav
#include "actionlib_msgs/GoalStatus.h"


#include <stdlib.h>
#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"

#include "robotino_msgs/DigitalReadings.h"
#include "sensor_msgs/LaserScan.h"

using namespace std::chrono_literals;

#define GRAMMAR_POCKET_COMMANDS "grammars/carryMyLuggage.jsgf"

enum SMState {
	SM_INIT,
	SM_FIND_BAG,
	SM_WAIT_FOR_BAG,
	SM_CONF_POINTING_HAND,
	SM_FIND_PERSON,
	SM_FOLLOW_OPERATOR,
	SM_WAIT_CONF_CAR,
	SM_LEAVE_BAG,
	SM_FIND_QUEUE,
	SM_NAV_QUEUE,
	SM_FOLLOW_QUEUE,
	SM_BACK_TO_ARENA,
    SM_FINAL_STATE
};

// Test results variables
static bool fail = false;
static bool success = false;

static bool legs_found = false;
static bool human_detector = false;
static geometry_msgs::Pose human_coordinates;
static geometry_msgs::PoseStamped tf_human_coordinates;

static SMState state = SM_INIT;

// Grammars
static std::string namesGrammar("help_me_carry_names.json");
static std::string interactionCommandsGrammar("help_me_carry_interaction_commands.json");
static std::string navigationCommandsGrammar("help_me_carry_navigation_commands.json");

static actionlib_msgs::GoalStatus simple_move_goal_status;
static int simple_move_status_id = 0;
static bool stop = false;
static int counter = 0;

static std::vector<float> goal_vec(3);

bool HumanDetector();
void humanBoolCallback(const std_msgs::msg::Bool::SharedPtr msg);
void humanCoordinatesCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
void transform_human_coordinates(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tfBuffer);
void callback_simple_move_goal_status(const actionlib_msgs::msg::GoalStatus::SharedPtr msg);


bool HumanDetector()
{
    return human_detector;
}

void humanBoolCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    human_detector = msg -> data; 
}

void humanCoordinatesCallback(const geometry_msgs::msg::Pose::SharedPtr msg)
{
    human_coordinates = *msg; 
}

void transform_human_coordinates(rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tfBuffer)
{
	std::string target = "hokuyo_laser_link"; 
    std::string source = "camera_link";       

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer->lookupTransform(
            target, source, tf2::TimePointZero, 1s);
    } catch (const std::exception &ex) {
        RCLCPP_ERROR(node->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    double tx = -transformStamped.transform.translation.x;
    double ty = -transformStamped.transform.translation.y;

    tf_human_coordinates.header.frame_id = "map";
    tf_human_coordinates.header.stamp = node->now();
    tf_human_coordinates.pose.position.x = tx * human_coordinates.position.x;
    tf_human_coordinates.pose.position.y = ty * human_coordinates.position.y;
    tf_human_coordinates.pose.position.z = 0.0;
    tf_human_coordinates.pose.orientation.w = 1.0;

	"""tf::TransformListener listener;
    tf::StampedTransform transform;

	try{
        std::cout << "Waiting for transform: " << std::endl;
        listener.waitForTransform("/hokuyo_laser_link", "/camera_link", ros::Time(0), ros::Duration(1000.0));
        listener.lookupTransform("/hokuyo_laser_link", "/camera_link", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    tf_human_coordinates.pose.position.x = -transform.getOrigin().x();
    tf_human_coordinates.pose.position.y = -transform.getOrigin().y();
    tf_human_coordinates.pose.position.x = tf_human_coordinates.pose.position.x * human_coordinates.position.x;
    tf_human_coordinates.pose.position.y = tf_human_coordinates.pose.position.y * human_coordinates.position.y;"""
}

void callback_simple_move_goal_status(const actionlib_msgs::msg::GoalStatus::SharedPtr msg)
{
    simple_move_goal_status = *msg;
	try{
	    std::stringstream ss;
		ss << msg->goal_id.id;
		ss >> simple_move_status_id;
	} catch (...){
	    simple_move_status_id = 0;
	}

}

int main(int argc, char** argv){
	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("carry_my_luggage_node");
	RCLCPP_INFO(node->get_logger(), "INITIALIZING CARRY MY LUGGAGE NODE... ");

	HRI::setNodeHandle(node);
	Vision::setNodeHandle(node);
	Navigation::setNodeHandle(node);
	bool latch;
	std::cout << "INITIALIZING CARRY MY LUGGAGE NODE... " << std::endl;
    rclcpp::init(argc, argv, "SM");
    rclcpp::NodeHandle n;
    HRI::setNodeHandle(&n);
    Vision::setNodeHandle(&n);
    Navigation::setNodeHandle(&n);
    Hardware::setNodeHandle(&n);

    rclcpp::Publisher pub_digital = n.advertise<robotino_msgs::DigitalReadings>("/set_digital_values", 1000);
    rclcpp::Subscriber sub_human_bool = n.subscribe("human_detector_bool", 1000, humanBoolCallback);
    rclcpp::Subscriber sub_human_coordinates = n.subscribe("human_detector_coordinates", 1000, humanCoordinatesCallback);
    rclcpp::Publisher pub_goal = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1000); 
    rclcpp::Subscriber sub_move_goal_status   = n.subscribe("/simple_move/goal_reached", 10, callback_simple_move_goal_status);

    rclcpp::Rate loop(30);

    // Interacion variables
    std::string voice;
    std::string recogSpeech;
    std::string left_arm_pose;
    std::string chosenDirection;
    std::vector<std::string> pointingDirections;

    //Open Pose variables
    bool pointing_hand;
    bool human_detector_bool;

    //Robotino Lights
    robotino_msgs::DigitalReadings arr_values;
    arr_values.stamp.sec = 0;
    arr_values.stamp.nsec = 0;
    arr_values.values = {0,0,0,0,0,0};
    //This delay is necessary
    HRI::say(" ",3);

    //TF related stuff
    tf_human_coordinates.header.frame_id = "/map";
    tf_human_coordinates.pose.position.x = 0.0;
    tf_human_coordinates.pose.position.y = 0.0;
    tf_human_coordinates.pose.position.z = 0.0;
    tf_human_coordinates.pose.orientation.x = 0.0;
    tf_human_coordinates.pose.orientation.y = 0.0;
    tf_human_coordinates.pose.orientation.z = 0.0;
    tf_human_coordinates.pose.orientation.w = 0.0;

    //Dict with coords pose,
// Navigation to Arena
	int pose_counter = 0;
	int save_every = 500;
    std::map<int, std::tuple<float, float, float>> poses;
    float currentX, currentY, currentTheta;
    float goalX, goalY, goalTheta;
    std::vector<float> goal_vec(3);


	while(rclcpp::ok() && !fail && !success){
	    switch(state){
			case SM_INIT:
	    		std::cout << "State machine: SM_INIT" << std::endl;
	    		
	    		// Move head to the origin 
	    		// Now it is called move head
                Hardware::setHeadOrientation(0.0, -0.3);

                // Dejar de extender el brazo
			    left_arm_pose = "default";
			    Hardware::setArmPose(left_arm_pose);

                // Cerrar gripper
			    Hardware::setGripperPose(-0.15);
                
                //Save initial pose with Map
			    Navigation::getRobotPoseWrtMap(currentX, currentY, currentTheta);
			    poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);

	            voice = "My name is Justina, I am ready for the help me carry test";
	            HRI::say(voice, 4);

//				voice = "Please tell me your name. I will use it while helping you";
//				FestinoHRI::say(voice, 5);

//	            voice = "Please point at the bag you would like me to carry";
//	    		FestinoHRI::say(voice, 4);

	    		state = SM_FIND_PERSON;

	    		break;

	    	case SM_FIND_BAG:
	    		std::cout << "State machine: SM_FIND_BAG" << std::endl;

                voice = "Say, Justina yes, once you are pointing at the bag";
				HRI::say(voice, 6);

	    	    recogSpeech = HRI::lastRecogSpeech(interactionCommandsGrammar);

                if(recogSpeech == "justina yes" || recogSpeech == "robot yes" || recogSpeech == "yes")
	    			state = SM_CONF_POINTING_HAND;
	    		
	    		break;

	    	case SM_CONF_POINTING_HAND:{
	    		std::cout << "State machine: SM_CONF_POINTING_HAND" << std::endl;
	    		
	    		Vision::enablePoseEstimation(true);
	    	
		        for (int i = 0; i < 10; ++i) {
		        
					std::string dir = Vision::PointingHand();
					pointingDirections.push_back(dir);
					std::cout << "PointingHand iteration " << i << ": " << dir << std::endl;
					
				}
		        
		        std::map<std::string, int> countMap;
				for (const std::string& dir : pointingDirections) {
			
			    	countMap[dir]++;
			
				}
			
				int maxCount = 0;
				for (const auto& pair : countMap) {
				
			    	if (pair.second > maxCount) {
			    	
					maxCount = pair.second;
					chosenDirection = pair.first;
				
			    	}
			    
				}
			
				std::cout << "Chosen direction: " << chosenDirection << std::endl;
			
				pointingDirections.clear();

	    		if (chosenDirection == "left"){

                    // Move head to the left
                    Hardware::setHeadOrientation(-0.5, -0.5);

    				voice = "Are you pointing at the bag on your left? Answer with, Justina yes, or, Justina no";
    				HRI::say(voice, 7);
    
                    recogSpeech = HRI::lastRecogSpeech(interactionCommandsGrammar);

    				if(recogSpeech == "justina yes" || recogSpeech == "robot yes" || recogSpeech == "yes"){

                        // Move head to the origin
	                    Hardware::setHeadOrientation(0.0, -0.3);

                        // Extender el brazo izquierdo
                        left_arm_pose = "pre_grasp";
			    		Hardware::setArmPose(left_arm_pose);

			    		// Abrir gripper
			    		Hardware::setGripperPose(0.5);
		    			
                        voice = "Please hang the bag on my left arm";
		            	HRI::say(voice, 8);
		            	    			
                        state = SM_WAIT_FOR_BAG;

    				}
	    		}

	    	    else if (chosenDirection == "right"){

                    // Move head to the right
                    Hardware::setHeadOrientation(0.5, -0.5);

    				voice = "Are you pointing at the bag on your right? Answer with, Justina yes, or, Justina no";
    				HRI::say(voice, 7);

                    recogSpeech = HRI::lastRecogSpeech(interactionCommandsGrammar);

    				if(recogSpeech == "justina yes" || recogSpeech == "robot yes" || recogSpeech == "yes"){

    					// Move head to the origin
                    	Hardware::setHeadOrientation(0.0, -0.3);

                        // Extender el brazo izquierdo
                        left_arm_pose = "pre_grasp";
			    		Hardware::setArmPose(left_arm_pose);

			    		// Abrir gripper
			    		Hardware::setGripperPose(0.5);

		    			voice = "Please hang the bag on my left arm";
                        HRI::say(voice, 8);

                        state = SM_WAIT_FOR_BAG;
    				
                    }
	    		}

                else if (chosenDirection == "none"){
                    
                    // Move head to the origin
                    Hardware::setHeadOrientation(0.0, -0.3);

                    voice = "I could not identify where you were pointing at";
    				HRI::say(voice, 4);

                    state = SM_FIND_BAG;

                }
                
                else{
                    
                    // Move head to the origin
                    Hardware::setHeadOrientation(0.0, -0.3);

                    voice = "I could not identify where you were pointing at";
    				HRI::say(voice, 4);

                    state = SM_FIND_BAG;

                }

	    		break;
	    	}

	    	case SM_WAIT_FOR_BAG:
	    		std::cout << "State machine: SM_WAIT_FOR_BAG" << std::endl;
	    		
	    		voice = "Tell me, Justina yes, once the bag was securely placed in my arm";
				HRI::say(voice, 6);

                recogSpeech = HRI::lastRecogSpeech(interactionCommandsGrammar);

	    		if(recogSpeech == "justina yes" || recogSpeech == "robot yes" || recogSpeech == "yes"){

                    // Cerrar gripper
			    	Hardware::setGripperPose(-0.15);

					state = SM_BACK_TO_ARENA;

	    		}

	    		break;

	    	case SM_FIND_PERSON:
				std::cout << "State machine: SM_FIND_PERSON" << std::endl;

                HRI::enableLegFinder(true); 

                legs_found = HRI::frontalLegsFound();
	    		std::cout << "Legs found: " << legs_found << std::endl;    

    			if(!legs_found){

	    			std::cout << "Not found legs" << std::endl;

	    			voice = "I could not find you, please stand infront of me";
					HRI::say(voice, 7);

	    			HRI::enableHumanFollower(false);

	    		}
    			else if(legs_found)
    			{

    				voice = "Say, Justina follow me, when you are ready";
					HRI::say(voice, 5);

                    recogSpeech = HRI::lastRecogSpeech(navigationCommandsGrammar);

		    		if(recogSpeech == "justina follow me" || recogSpeech == "robot follow me" || recogSpeech == "follow me"){

		    		    voice = "I'm going to follow you, please say, Justina we arrived, once we reach the final destination";
						HRI::say(voice, 7);
		    				
                        state = SM_FOLLOW_OPERATOR;
                    }    	
	    		}

	    		break;
	    	

	    	case SM_FOLLOW_OPERATOR:
	    	    std::cout << "State machine: SM_FOLLOW_OPERATOR" << std::endl;	

                HRI::enableLegFinder(true);
			    HRI::enableHumanFollower(true);
			    
			    stop = false;

				// Bucle para capturar las poses
			    while (!stop) {
			    
			    	 std::cout << "In loop" << std::endl;
                    
                    // Obtener la pose actual
					Navigation::getRobotPoseWrtMap(currentX, currentY, currentTheta);

					// Guardar cada N poses
					if (pose_counter % save_every == 0) {
					
						poses[pose_counter] = std::make_tuple(currentX, currentY, currentTheta);	
            			std::cout << "Saved pose " << pose_counter << ": x = " << currentX << ", y = " << currentY << ", theta = " << currentTheta << std::endl;
					
					}
			        
                    pose_counter++;
                    
                    std::cout << "Antes" << std::endl;
                    
                    recogSpeech = HRI::lastRecogSpeech(navigationCommandsGrammar);

					std::cout << "Despues" << std::endl;
			        
			        // Comprobar si se ha perdido al operador
                    legs_found = HRI::frontalLegsFound(); 
	    		    std::cout << "Legs found: " << legs_found << std::endl;    
                   
                    if (!legs_found) {

			            std::cout << "Lost operator" << std::endl;
			            
                        voice = "I lost you, please stand infront of me";
			            HRI::say(voice, 7);
			            
                        HRI::enableHumanFollower(false);
			            
                        state = SM_FIND_PERSON;
			            
                        stop = true;

			        }

			        else if(recogSpeech == "justina we arrived" || recogSpeech == "robot we arrived" || recogSpeech == "we arrived"){

                        std::cout << "We arrived" << std::endl;

                        HRI::enableHumanFollower(false);

			            state = SM_WAIT_CONF_CAR;

                        stop = true;
			        }

			     }

			        			
	    		break;

	    	case SM_WAIT_CONF_CAR:
				voice = "Is this your car? Say, Justina yes, or, Justina no";
				HRI::say(voice, 6);

		        recogSpeech = HRI::lastRecogSpeech(interactionCommandsGrammar);

				if(recogSpeech == "justina yes" || recogSpeech == "robot yes" || recogSpeech == "yes"){
								
					std::cout << "Here is the car" << std::endl;
					
                    HRI::enableLegFinder(false);
					HRI::enableHumanFollower(false);
                    
                    state = SM_FIND_BAG;

				}

				else{

					state = SM_FIND_PERSON;

				}

				break;

	    	case SM_LEAVE_BAG:
	    		std::cout << "State machine: SM_LEAVE_BAG" << std::endl;	
                    
                   // Dejar de extender el brazo
			        left_arm_pose = "default";
			        Hardware::setArmPose(left_arm_pose);


                    // Abrir gripper
			        Hardware::setGripperPose(0.5);

                    voice = "I hope I had been usefull";
				    HRI::say(voice, 3);

			        // Cerrar gripper
			        //FestinoHardware::setGripperPose(-0.15);

					state = SM_FINAL_STATE;                              

	    		break;

	    	case SM_BACK_TO_ARENA:
	    		std::cout<<"State: Back to arena"<<std::endl;
				
                voice = "I am going to return to my start position";
	    		HRI::say(voice, 5);

                // Recorrer las poses en orden inverso
				for (int i = pose_counter - 1; i >= 0; --i) {
				
					std::tie(goalX, goalY, goalTheta) = poses[i];

					// Sumar 180° (π rad) y normalizar entre -π y π
					//goalTheta += M_PI;
					//if (goalTheta > M_PI) goalTheta -= 2 * M_PI;
					//if (goalTheta < -M_PI) goalTheta += 2 * M_PI;

					std::cout << "Navigating to: x = " << goalX << "; y = " << goalY << "; theta = " << goalTheta << std::endl;

					if (!Navigation::getClose(goalX, goalY, goalTheta, 30000)) {
						if (!Navigation::getClose(goalX, goalY, goalTheta, 30000)){
						
						    std::cout << "Cannot move to point" << std::endl;
						    Navigation::stopNavigation();
		        			HRI::enableHumanFollower(false);

							voice = "I have reached my final destination";
							HRI::say(voice, 5);

							state = SM_LEAVE_BAG;

						}
						
					}
				}

		        Navigation::stopNavigation();
		        HRI::enableHumanFollower(false);

		        voice = "I have reached my final destination";
	    		HRI::say(voice, 5);

		        state = SM_LEAVE_BAG;
	    		
                break;	

	    	case SM_FIND_QUEUE:
	    		std::cout << "State machine: SM_FIND_QUEUE" << std::endl;
	    		rclcpp::Duration(2, 0).sleep();

	    		/*transform_human_coordinates();
	    		pub_goal.publish(tf_human_coordinates);
	    		
	    		state = SM_NAV_QUEUE;*/

	    		HRI::enableLegFinder(true);

    			if(!HRI::frontalLegsFound()){
	    			
                    std::cout << "Not found" << std::endl;
	    			HRI::enableHumanFollower(false);
	    		
                }

    			else{

    				voice = "Im going to follow the queue";
	            	HRI::say(voice, 2);
	    			state = SM_FOLLOW_QUEUE;
	    		
                }

	    		break;

	    	case SM_NAV_QUEUE:
            	std::cout << "State machine: SM_NAV_QUEUE" << std::endl;
	            
                voice = "Navigating to queue";
	    		HRI::say(voice, 2);

	            if(simple_move_goal_status.status == actionlib_msgs::GoalStatus::SUCCEEDED && simple_move_status_id == -1){
	            
                    voice = "Goal location reached";
	                HRI::say(voice, 2);

	                HRI::enableLegFinder(true);
					HRI::enableHumanFollower(true);

                    state = SM_FOLLOW_QUEUE;

                }

                break;
	        
	    	case SM_FOLLOW_QUEUE:
	    		std::cout << "State machine: SM_FOLLOW_QUEUE" << std::endl;

	    		HRI::enableHumanFollower(true);
	    		rclcpp::Duration(200, 0).sleep();

	    		HRI::enableHumanFollower(false);

	    		state = SM_FINAL_STATE;

	    		break;

	    	case SM_FINAL_STATE:
	    		std::cout << "State machine: SM_FINAL_STATE" << std::endl;	

	            voice =  "I have finished the test";
	            HRI::say(voice, 3);

	    		success = true;
	    		fail = true;

	    		break;
		}

	    rclcpp::spinOnce();
	    loop.sleep();

	}

	return 0;

}
