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

#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"
#include "knowledge_tools/KnowledgeTools.hpp"

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
                "executor_worker_node ready (Listen, Say, Nav, DoorCheck & Save are Ready)");
  }

  /* ===================== INIT TOOLS ===================== */

  void initTools() {
    auto nodePtr = shared_from_this();

    // UNA sola instancia de HRI
    hri = std::make_shared<HRI>(nodePtr);

    // Instancia de navigation
    navigation = std::make_shared<Navigation>(nodePtr);

    // Instancia de Knowledge
    knowledge = std::make_shared<KnowledgeTools>(nodePtr);

    // Instancia de Hardware
    hardware = std::make_shared<Hardware>(nodePtr);

    // Instancias de executor_tools
    listen = std::make_shared<Listen>(nodePtr, hri);
    say = std::make_shared<Say>(nodePtr, hri);
    go_to = std::make_shared<GoTo>(nodePtr, navigation);
    check = std::make_shared<Check>(nodePtr, hardware);
    save = std::make_shared<Save>(nodePtr, knowledge);

    actionMap["listen_for"] =
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
    actionMap["wait_for"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
          return check->run(arg, gh);
        };
    actionMap["save"] =
        [this](const std::string &arg,
               const std::shared_ptr<GoalHandleExecuteTask> &gh) {
          return save->run(arg, gh);
        };
  }

private:
  /* ===================== ACTION CALLBACKS ===================== */

  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const ExecuteTask::Goal> goal) {
    const auto &step = goal->task;

    if (actionMap.find(step.action) == actionMap.end()) {
      RCLCPP_WARN(get_logger(), "Rejected unknown action: %s",
                  step.action.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

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
      success = false;
      message = "Unknown action";
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
  std::shared_ptr<KnowledgeTools> knowledge;
  std::shared_ptr<Listen> listen;
  std::shared_ptr<Say> say;
  std::shared_ptr<GoTo> go_to;
  std::shared_ptr<Check> check;
  std::shared_ptr<Save> save;

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
