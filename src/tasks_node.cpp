#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"

#include "navigation_tools/Navigation.hpp"
#include "hri_tools/HRI.hpp"
#include "hardware_tools/Hardware.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TasksNode : public rclcpp::Node
{
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

  TasksNode()
  : Node("tasks_node")
  {
    action_server_ = rclcpp_action::create_server<ExecuteTask>(
      this,
      "/executor/execute_task",
      std::bind(&TasksNode::handle_goal, this, _1, _2),
      std::bind(&TasksNode::handle_cancel, this, _1),
      std::bind(&TasksNode::handle_accepted, this, _1)
    );

    RCLCPP_INFO(
      get_logger(),
      "tasks_node ready. Action server: /executor/execute_task");
  }

  void init_tools()
  {
    aux_node_ = rclcpp::Node::make_shared("tasks_node_hardware_client");
    nav_ = std::make_shared<Navigation>(shared_from_this());
    hri_ = std::make_shared<HRI>(shared_from_this());
    hardware_ = std::make_shared<Hardware>(aux_node_); 
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteTask::Goal> goal)
  {
    const auto & step = goal->task;

    static const std::set<std::string> valid_actions =
      {"goto", "say", "listen", "check"};

    if (valid_actions.find(step.action) == valid_actions.end())
    {
      RCLCPP_WARN(
        get_logger(),
        "Goal rejected: unknown action '%s'",
        step.action.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (step.action == "goto" &&
        (step.args.empty() || step.args[0].empty()))
      return rclcpp_action::GoalResponse::REJECT;

    if (step.action == "say" && step.args.empty())
      return rclcpp_action::GoalResponse::REJECT;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask>)
  {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    std::thread(
      &TasksNode::execute,
      this,
      goal_handle
    ).detach();
  }

  void execute(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
  {
    try
    {
      cancel_requested_.store(false);

      const auto goal = goal_handle->get_goal();
      const auto & step = goal->task;
      const uint32_t step_id = step.step_id;

      bool success = false;
      std::string message;

      RCLCPP_INFO(
        get_logger(),
        "Executing action '%s' (step_id=%u)",
        step.action.c_str(), step_id);

      if (step.action == "goto")
      {
        const std::string & location = step.args[0];
        publish_feedback(goal_handle, step_id, "Moving to " + location);

        success = nav_->GoTo(location);
        message = success ? "Arrived to " + location : "Failed goto " + location;
      }
      else if (step.action == "say")
      {
        const std::string & text = step.args[0];
        publish_feedback(goal_handle, step_id, "Saying: " + text);

        success = hri_->Say(text);
        message = success ? "Finished saying" : "Failed to say";
      }
      else if (step.action == "listen")
      {
        const std::string grammar = step.args.empty() ? "" : step.args[0];
        publish_feedback(goal_handle, step_id, "Listening...");

        const std::string heard = hri_->Listen(grammar);

        if (!heard.empty())
        {
          success = true;
          message = "Heard: " + heard;
          RCLCPP_INFO(get_logger(), "HRI heard: %s", heard.c_str());
        }
        else
        {
          success = false;
          message = "Listen failed or timed out";
        }
      }
      else if (step.action == "check")
      {
        publish_feedback(goal_handle, step_id, "Checking door status...");
        success = hardware_->CheckDoor();

        if (success) {
            message = "Check successful: Door is OPEN";
            RCLCPP_INFO(get_logger(), "Door is OPEN");
        } else {
            message = "Check failed: Door is CLOSED or Service Error";
            RCLCPP_WARN(get_logger(), "Door check failed");
        }
      }
      auto result = std::make_shared<ExecuteTask::Result>();
      result->result.step_id = step_id;
      result->result.message = message;

      if (goal_handle->is_canceling() || cancel_requested_.load())
      {
        result->result.status = executor_interfaces::msg::TaskResult::ABORTED;
        goal_handle->canceled(result);
        return;
      }

      if (success)
      {
        result->result.status = executor_interfaces::msg::TaskResult::SUCCESS;
        goal_handle->succeed(result);
      }
      else
      {
        result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
        goal_handle->abort(result);
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(get_logger(), "Exception in task execution: %s", e.what());

      auto result = std::make_shared<ExecuteTask::Result>();
      result->result.step_id = goal_handle->get_goal()->task.step_id;
      result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
      result->result.message = e.what();

      goal_handle->abort(result);
    }
  }

  void publish_feedback(
    const std::shared_ptr<GoalHandleExecuteTask>& goal_handle,
    uint32_t step_id,
    const std::string& msg)
  {
    auto fb = std::make_shared<ExecuteTask::Feedback>();
    fb->feedback.step_id = step_id;
    fb->feedback.progress = 0.5f;
    fb->feedback.message = msg;
    goal_handle->publish_feedback(fb);
  }

private:
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;
  rclcpp::Node::SharedPtr aux_node_; 

  std::shared_ptr<Navigation> nav_;
  std::shared_ptr<HRI> hri_;
  std::shared_ptr<Hardware> hardware_;

  std::atomic<bool> cancel_requested_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TasksNode>();
  node->init_tools();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}