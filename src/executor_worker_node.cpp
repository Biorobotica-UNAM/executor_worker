#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"

#include "executor_tools/Check.hpp"
#include "executor_tools/GoTo.hpp"
#include "executor_tools/Listen.hpp"
#include "executor_tools/Say.hpp"
#include "executor_tools/Save.hpp"
#include "executor_tools/Follow.hpp"
#include "executor_tools/Executor.hpp" 
#include "executor_tools/MemorizeFace.hpp" 
#include "executor_tools/Wait.hpp"

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
                "executor_worker_node ready (Listen, Say, Nav, Follow, Check, Save, Wait Ready)");
  }

  /* ===================== INIT TOOLS ===================== */

  void initTools() {
    auto nodePtr = shared_from_this();

    // Definimos las funciones en desarrollo
    std::vector<std::string> dev = {"focus", "find", "point", "take", "align", "drop"};

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

    // Instancias de executor_tools
    listen = std::make_shared<Listen>(nodePtr, hri, knowledge);
    say = std::make_shared<Say>(nodePtr, hri);
    go_to = std::make_shared<GoTo>(nodePtr, navigation);
    check = std::make_shared<Check>(nodePtr, hardware);
    save = std::make_shared<Save>(nodePtr, knowledge);
    follow = std::make_shared<Follow>(nodePtr);
    memorize_face = std::make_shared<MemorizeFace>(nodePtr);
    wait = std::make_shared<Wait>(nodePtr, vision);

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
          // Resolvemos el label (ej: "name_1" -> "john")
          std::string real_name = knowledge->getNameFromLabel(arg);
          return memorize_face->run(real_name, gh);
        };
    actionMap["wait"] = 
        [this](const std::string &arg, 
                const std::shared_ptr<GoalHandleExecuteTask> &gh) {
          return wait->run(arg, gh);
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
    const auto goal = goalHandle->get_goal();
    const auto &step = goal->task;

    const std::string argument = step.args.empty() ? "" : step.args[0];

    bool success = false;
    std::string message;

    auto it = actionMap.find(step.action);

    if (it != actionMap.end()) {
      success = it->second(argument, goalHandle);
      message = success ? "Execution authorized" : "Execution rejected";
    } else {
      // Si la acción no es válida, mandamos el authorize con los parámetros que pediste
      Executor report_executor(shared_from_this());
      
      report_executor.authorize(
          step.action,
          argument,
          "reject",
          "Action not valid: " + step.action
      );

      success = false;
      message = "Unknown action: " + step.action;
    }

    auto result = std::make_shared<ExecuteTask::Result>();

    result->result.step_id = step.step_id;
    result->result.message = message;

    if (success) {
      result->result.status = executor_interfaces::msg::TaskResult::SUCCESS;
      goalHandle->succeed(result);
    } else {
      result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
      goalHandle->abort(result);
    }
  }

  /* ===================== MEMBERS ===================== */

  rclcpp_action::Server<ExecuteTask>::SharedPtr actionServer;

  std::shared_ptr<HRI> hri;
  std::shared_ptr<Navigation> navigation;
  std::shared_ptr<Hardware> hardware;
  std::shared_ptr<Knowledge> knowledge;
  std::shared_ptr<Vision> vision;
  std::shared_ptr<Listen> listen;
  std::shared_ptr<Say> say;
  std::shared_ptr<GoTo> go_to;
  std::shared_ptr<Check> check;
  std::shared_ptr<Save> save;
  std::shared_ptr<Follow> follow;
  std::shared_ptr<MemorizeFace> memorize_face;
  std::shared_ptr<Wait> wait;

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