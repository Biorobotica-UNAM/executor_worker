#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"

#include "executor_tools/Listen.hpp"
#include "executor_tools/Say.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ExecutorWorkerNode : public rclcpp::Node
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask =
    rclcpp_action::ServerGoalHandle<ExecuteTask>;

  ExecutorWorkerNode()
  : Node("executor_worker_node")
  {
    action_server_ =
      rclcpp_action::create_server<ExecuteTask>(
        this,
        "/executor/execute_task",
        std::bind(&ExecutorWorkerNode::handle_goal, this, _1, _2),
        std::bind(&ExecutorWorkerNode::handle_cancel, this, _1),
        std::bind(&ExecutorWorkerNode::handle_accepted, this, _1)
      );

    init_tools();

    RCLCPP_INFO(
      get_logger(),
      "executor_worker_node ready (Listen & Say only)");
  }

private:

  /* ===================== INIT TOOLS ===================== */

  void init_tools()
  {
    listen_ = std::make_shared<Listen>(shared_from_this());
    say_    = std::make_shared<Say>(shared_from_this());

    action_map_["listen"] =
      [this](const std::string& arg,
             const std::shared_ptr<GoalHandleExecuteTask>& gh)
      {
        return listen_->run(arg, gh);
      };

    action_map_["say"] =
      [this](const std::string& arg,
             const std::shared_ptr<GoalHandleExecuteTask>& gh)
      {
        return say_->run(arg, gh);
      };
  }

  /* ===================== ACTION CALLBACKS ===================== */

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTask::Goal> goal)
  {
    const auto & step = goal->task;

    if (action_map_.find(step.action) == action_map_.end())
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejected unknown action: %s",
        step.action.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    std::thread(
      &ExecutorWorkerNode::execute,
      this,
      goal_handle).detach();
  }

  /* ===================== EXECUTION ===================== */

  void execute(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const auto & step = goal->task;

    const std::string argument =
      step.args.empty() ? "" : step.args[0];

    bool success = false;
    std::string message;

    auto it = action_map_.find(step.action);

    if (it != action_map_.end())
    {
      success = it->second(argument, goal_handle);
      message = success ?
        "Execution authorized" :
        "Execution rejected";
    }
    else
    {
      success = false;
      message = "Unknown action";
    }

    auto result =
      std::make_shared<ExecuteTask::Result>();

    result->result.step_id = step.step_id;
    result->result.message = message;

    if (success)
    {
      result->result.status =
        executor_interfaces::msg::TaskResult::SUCCESS;
      goal_handle->succeed(result);
    }
    else
    {
      result->result.status =
        executor_interfaces::msg::TaskResult::FAILURE;
      goal_handle->abort(result);
    }
  }

  /* ===================== MEMBERS ===================== */

  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

  std::shared_ptr<Listen> listen_;
  std::shared_ptr<Say> say_;

  using ActionFunction =
    std::function<bool(
      const std::string&,
      const std::shared_ptr<GoalHandleExecuteTask>&)>;

  std::unordered_map<std::string, ActionFunction> action_map_;
};


/* ===================== MAIN ===================== */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ExecutorWorkerNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

