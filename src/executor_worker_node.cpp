#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/srv/update_knowledge.hpp"

#include "executor_tools/Align.hpp"
#include "executor_tools/Check.hpp"
#include "executor_tools/GoTo.hpp"
#include "executor_tools/Listen.hpp"
#include "executor_tools/Say.hpp"
#include "executor_tools/Save.hpp"
#include "executor_tools/Follow.hpp"
#include "executor_tools/Executor.hpp" 
#include "executor_tools/MemorizeFace.hpp" 
#include "executor_tools/Find.hpp"
#include "executor_tools/Point.hpp"
#include "executor_tools/Describe.hpp"
#include "executor_tools/Focus.hpp"
#include "executor_tools/Unfocus.hpp"
#include "executor_tools/Saveloc.hpp"
#include "executor_tools/Take.hpp"
#include "executor_tools/FindObject.hpp"
#include "executor_tools/FindPerson.hpp"
#include "executor_tools/FindSeat.hpp"
#include "executor_tools/FindWaving.hpp"

#include "tasks_tools/Tasks.hpp" 

#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"
#include "vision_tools/Vision.hpp"
#include "executor_tools/Knowledge.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ExecutorWorkerNode : public rclcpp::Node {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  ExecutorWorkerNode() : Node("executor_worker_node") {
    actionServer = rclcpp_action::create_server<ExecuteTask>(
        this, "/executor/execute_task",
        std::bind(&ExecutorWorkerNode::handleGoal, this, _1, _2),
        std::bind(&ExecutorWorkerNode::handleCancel, this, _1),
        std::bind(&ExecutorWorkerNode::handleAccepted, this, _1));

    RCLCPP_INFO(get_logger(),
                "executor_worker_node ready (Listen, Say, Nav, Follow, Check, Save, Find Ready)");
  }

  /* ===================== INIT TOOLS ===================== */

  void initTools() {
    auto nodePtr = shared_from_this();

    // Definimos las funciones en desarrollo
    std::vector<std::string> dev = { "drop"};

    // UNA sola instancia de HRI
    hri = std::make_shared<HRI>(nodePtr);

    // Instancia de navigation
    navigation = std::make_shared<Navigation>(nodePtr);

    // Instancia de Knowledge
    knowledge = std::make_shared<Knowledge>(nodePtr);

    // Instancia de Hardware
    hardware = std::make_shared<Hardware>(nodePtr);

    // Instancia de Vision
    vision = std::make_shared<Vision>(nodePtr);

    // Instancia de Tasks
    tasks = std::make_shared<Tasks>(nodePtr);

    // Instancias de executor_tools
    listen = std::make_shared<Listen>(nodePtr, hri, knowledge);
    say = std::make_shared<Say>(nodePtr, hri, knowledge);
    go_to = std::make_shared<GoTo>(nodePtr, navigation);
    check = std::make_shared<Check>(nodePtr, hardware);
    save = std::make_shared<Save>(nodePtr, knowledge);
    follow = std::make_shared<Follow>(nodePtr);
    memorize_face = std::make_shared<MemorizeFace>(nodePtr, vision, knowledge);
    find = std::make_shared<Find>(nodePtr, tasks,knowledge);
    point_tool = std::make_shared<Point>(nodePtr, hardware);
    describe_tool = std::make_shared<Describe>(nodePtr, knowledge);
    focus_tool = std::make_shared<Focus>(nodePtr);
	unfocus_tool = std::make_shared<Unfocus>(nodePtr);
    saveloc = std::make_shared<Saveloc>(nodePtr);
    take_tool = std::make_shared<Take>(nodePtr, knowledge);
    align_tool = std::make_shared<Align>(nodePtr);
    find_object_tool = std::make_shared<FindObject>(nodePtr, tasks, knowledge);
    find_person_tool = std::make_shared<FindPerson>(nodePtr, tasks, knowledge);
    find_seat_tool   = std::make_shared<FindSeat>(nodePtr, tasks, knowledge);
    find_waving_tool = std::make_shared<FindWaving>(nodePtr, tasks, knowledge);

    actionMap["listen"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return listen->run(arg, gh);
        };

    actionMap["say"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return say->run(arg, gh);
        };

    actionMap["goto"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return go_to->run(arg, gh);
        };
    actionMap["check"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return check->run(arg, gh);
        };
    actionMap["save"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return save->run(arg, gh);
        };
    actionMap["follow"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return follow->run(arg, gh);
        };
    actionMap["memorize_face"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return memorize_face->run(arg, gh);
        };
    actionMap["find"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return find->run(arg, gh);
        };
    actionMap["point"] =   
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return point_tool->run(arg, gh);
        };
    actionMap["describe"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return describe_tool->run(arg, gh);
        };
	actionMap["focus"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return focus_tool->run(arg, gh);
        };
	actionMap["unfocus"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return unfocus_tool->run(arg, gh);
        };
    actionMap["saveloc"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return saveloc->run(arg, gh);
        };
    actionMap["take"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return take_tool->run(arg, gh);
        };
    actionMap["align"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return align_tool->run(arg, gh);
        };
    actionMap["find_object"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return find_object_tool->run(arg, gh);
        };
    actionMap["find_person"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return find_person_tool->run(arg, gh);
        };
    actionMap["find_seat"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return find_seat_tool->run(arg, gh);
        };
    actionMap["find_waving"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
        return find_waving_tool->run(arg, gh);
        };

    /* ===================== Registro automático para funciones en desarrollo ===================== */
    for (const auto& tool_name : dev) {
        actionMap[tool_name] = 
            [this, tool_name](const std::string &arg, 
                   const std::shared_ptr<GoalHandleExecuteTask> &gh) {
            RCLCPP_INFO(this->get_logger(), "Executing placeholder for: %s", tool_name.c_str());

            // 1. Feedback rápido
            auto feedback = std::make_shared<ExecuteTask::Feedback>();
            feedback->feedback.message = "Tool '" + tool_name + "' is under development...";
            gh->publish_feedback(feedback);

            // 2. Reporte al Supervisor usando la estructura de Executor
            Executor report_executor(shared_from_this());
            // Avisamos en el mensaje que es un placeholder.
            return report_executor.authorize(
                tool_name, 
                arg, 
                "success", 
                "PLACEHOLDER: " + tool_name + " not implemented yet."
            );
        };
    }

    update_knowledge_service = this->create_service<executor_interfaces::srv::UpdateKnowledge>(
        "/knowledge/update_label",
        [this](const std::shared_ptr<executor_interfaces::srv::UpdateKnowledge::Request> request,
              std::shared_ptr<executor_interfaces::srv::UpdateKnowledge::Response> response) {
            
            RCLCPP_INFO(this->get_logger(), "External update request: %s = %s", 
                        request->label.c_str(), request->value.c_str());
            
            // Llamamos a la nueva función que pusimos en Knowledge.hpp
            response->success = knowledge->updateFromService(request->label, request->value);
            
            if (response->success) {
                response->message = "Label " + request->label + " updated in RAM and Disk";
            } else {
                response->message = "Failed to update label";
            }
        }
    );
  }

private:
  /* ===================== ACTION CALLBACKS ===================== */

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const ExecuteTask::Goal> goal) {
    // Aceptamos la meta para poder procesar el error y mandar authorize en execute()
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandleExecuteTask>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleExecuteTask> goalHandle) {
    std::thread(&ExecutorWorkerNode::execute, this, goalHandle).detach();
  }

  /* ===================== EXECUTION ===================== */

  void execute(const std::shared_ptr<GoalHandleExecuteTask> goalHandle) {
    // 1. Verificación de seguridad inicial
    if (!goalHandle) {
        RCLCPP_ERROR(this->get_logger(), "Received a null goalHandle. Aborting execution.");
        return;
    }

    const auto goal = goalHandle->get_goal();
    const auto &step = goal->task;
    const std::string argument = step.args.empty() ? "" : step.args[0];

    bool success = false;
    std::string message;

    // 2. Envolvemos TODO en un try-catch para atrapar el SegFault antes de que mate al proceso
    try {
        auto it = actionMap.find(step.action);

        if (it != actionMap.end()) {
            RCLCPP_INFO(this->get_logger(), "Executing action: %s", step.action.c_str());
            
            // Llamada a la Tool (Find, Listen, etc.)
            success = it->second(argument, goalHandle);
            
            message = success ? "Execution authorized" : "Execution rejected/failed";
        } else {
            // Acción no válida
            Executor report_executor(shared_from_this());
            report_executor.authorize(step.action, argument, "reject", "Action not valid: " + step.action);
            
            success = false;
            message = "Unknown action: " + step.action;
        }
    } 
    catch (const std::exception &e) {
        // SI ALGO TRUENA DENTRO DE LA TOOL, CAE AQUÍ Y EL WORKER SIGUE VIVO
        RCLCPP_ERROR(this->get_logger(), "CRITICAL ERROR in Tool '%s': %s", step.action.c_str(), e.what());
        success = false;
        message = "Tool crashed internaly: " + std::string(e.what());
    }
    catch (...) {
        // Atrapa errores raros que no son excepciones estándar
        RCLCPP_ERROR(this->get_logger(), "UNKNOWN CRITICAL ERROR in Tool '%s'", step.action.c_str());
        success = false;
        message = "Tool caused an unknown memory error.";
    }

    // 3. Responder al Action Client (Supervisor) de forma segura
    auto result = std::make_shared<ExecuteTask::Result>();
    result->result.step_id = step.step_id;
    result->result.message = message;

    // Verificamos si el goalHandle sigue activo antes de responder
    if (goalHandle->is_active()) {
        if (success) {
            result->result.status = executor_interfaces::msg::TaskResult::SUCCESS;
            goalHandle->succeed(result);
        } else {
            result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
            goalHandle->abort(result);
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "GoalHandle is no longer active, skipping result publish.");
    }
  }

  /* ===================== MEMBERS ===================== */

  rclcpp_action::Server<ExecuteTask>::SharedPtr actionServer;
  rclcpp::Service<executor_interfaces::srv::UpdateKnowledge>::SharedPtr update_knowledge_service;

  std::shared_ptr<HRI> hri;
  std::shared_ptr<Navigation> navigation;
  std::shared_ptr<Hardware> hardware;
  std::shared_ptr<Knowledge> knowledge;
  std::shared_ptr<Vision> vision;
  std::shared_ptr<Tasks> tasks;
  std::shared_ptr<Listen> listen;
  std::shared_ptr<Say> say;
  std::shared_ptr<GoTo> go_to;
  std::shared_ptr<Check> check;
  std::shared_ptr<Save> save;
  std::shared_ptr<Follow> follow;
  std::shared_ptr<MemorizeFace> memorize_face;
  std::shared_ptr<Find> find;
  std::shared_ptr<Point> point_tool;
  std::shared_ptr<Describe> describe_tool;
  std::shared_ptr<Focus> focus_tool;
  std::shared_ptr<Unfocus> unfocus_tool;
  std::shared_ptr<Saveloc> saveloc;
  std::shared_ptr<Take> take_tool;
  std::shared_ptr<Align> align_tool;
  std::shared_ptr<FindObject> find_object_tool;
  std::shared_ptr<FindPerson> find_person_tool;
  std::shared_ptr<FindSeat> find_seat_tool;
  std::shared_ptr<FindWaving> find_waving_tool;

  using ActionFunction = std::function<bool(
      const std::string &, const std::shared_ptr<GoalHandleExecuteTask> &)>;

  std::unordered_map<std::string, ActionFunction> actionMap;
};

/* ===================== MAIN ===================== */

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ExecutorWorkerNode>();

  node->initTools();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
