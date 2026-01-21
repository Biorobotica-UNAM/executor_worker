#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"

// --- Tools
#include "navigation_tools/Navigation.hpp"

class TaskNode : public rclcpp::Node
{
  public:
    using ExecuteTask = executor_interfaces::action::ExecuteTask;
    using GoalHandleExecuteTask = rclcpp_action::ServerGoalHandle<ExecuteTask>;

    TaskNode() : Node("task_node")
    {
      const std::string action_name = "execute_task";

      action_server_ = rclcpp_action::create_server<ExecuteTask>(
        this,
        action_name,
        std::bind(&TaskNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&TaskNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&TaskNode::handle_accepted, this, std::placeholders::_1)
      );
      RCLCPP_INFO(get_logger(), "task_node listo. Action server: /%s", action_name.c_str());
    }

    void init_tools()
    {
      nav_ = std::make_shared<Navigation>(shared_from_this());
    }

  private:
    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    
    std::shared_ptr<const ExecuteTask::Goal> goal)
    {
      // Validación mínima: solo aceptamos goto
      const auto & step = goal->task;

      if (step.action != "goto")
      {
        RCLCPP_WARN(get_logger(), "Goal declined: action '%s' not soported (for now, just 'goto')", step.action.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      if (step.args.empty() || step.args[0].empty()) {
        RCLCPP_WARN(get_logger(), "Goal declined: goto requires args[0]=location_name");
        return rclcpp_action::GoalResponse::REJECT;
      }

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
      (void)goal_handle;
      cancel_requested_.store(true);
      RCLCPP_WARN(get_logger(), "Cancel solicitado para la tarea actual (se reflejará al terminar).");
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
      std::thread{std::bind(&TaskNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
      cancel_requested_.store(false);

      const auto goal = goal_handle->get_goal();
      const auto step = goal->task;

      const uint32_t step_id = step.step_id;
      const std::string location = step.args[0];

      {
        auto fb = std::make_shared<ExecuteTask::Feedback>();
        fb->feedback.step_id = step_id;
        fb->feedback.progress = 0.0f;
        fb->feedback.message = "Starting goto to " + location;
        goal_handle->publish_feedback(fb);
      }

      RCLCPP_INFO(get_logger(), "Executing goto (step_id=%u) -> %s",
                  step_id, location.c_str());

      bool ok = nav_->GoTo(location);

      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        auto result = std::make_shared<ExecuteTask::Result>();
        result->result.step_id = step_id;
        result->result.status = executor_interfaces::msg::TaskResult::ABORTED;
        result->result.message = "Canceled: goto " + location;

        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Task canceled (step_id=%u).", step_id);
        return;
      }

      {
        auto fb = std::make_shared<ExecuteTask::Feedback>();
        fb->feedback.step_id = step_id;
        fb->feedback.progress = 1.0f;
        fb->feedback.message = ok ? ("Arrived to " + location) : ("Failed goto " + location);
        goal_handle->publish_feedback(fb);
      }

      auto result = std::make_shared<ExecuteTask::Result>();
      result->result.step_id = step_id;

      if (ok) {
        result->result.status = executor_interfaces::msg::TaskResult::SUCCESS;
        result->result.message = "SUCCESS: goto " + location;
        goal_handle->succeed(result);
        RCLCPP_INFO(get_logger(), "SUCCESS (step_id=%u) goto %s", step_id, location.c_str());
      } else {
        result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
        result->result.message = "FAILURE: goto " + location;
        goal_handle->abort(result);
        RCLCPP_WARN(get_logger(), "FAILURE (step_id=%u) goto %s", step_id, location.c_str());
      }
    }

  private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

    std::shared_ptr<Navigation> nav_;
    std::atomic<bool> cancel_requested_{false};
  };

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskNode>();
  node->init_tools();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
