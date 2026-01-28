#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <set>  // <--- ¡ESTA ERA LA QUE FALTABA!

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"

// --- Tools
#include "navigation_tools/Navigation.hpp"
#include "hri_tools/HRI.hpp"

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
      RCLCPP_INFO(get_logger(), "task_node ready. Action server: /%s", action_name.c_str());
    }

    void init_tools()
    {
      nav_ = std::make_shared<Navigation>(shared_from_this());
      hri_ = std::make_shared<HRI>(shared_from_this());
    }

  private:
    rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ExecuteTask::Goal> goal)
    {
      // Validación mínima: solo aceptamos goto, speak y listen
      const auto & step = goal->task;
      
      // Necesitas #include <set> para que esto funcione
      std::set<std::string> valid_actions = {"goto", "speak", "listen"};

      if (valid_actions.find(step.action) == valid_actions.end())
      {
        RCLCPP_WARN(get_logger(), "Goal declined: action '%s' unknown", step.action.c_str());
        return rclcpp_action::GoalResponse::REJECT;
      }

      // Validaciones básicas de argumentos
      if (step.action == "goto" && (step.args.empty() || step.args[0].empty())) {
        return rclcpp_action::GoalResponse::REJECT;
      }
      if (step.action == "speak" && step.args.empty()) {
         return rclcpp_action::GoalResponse::REJECT;
      }
      // 'listen' es flexible con argumentos

      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleExecuteTask> goal_handle)
    {
      (void)goal_handle;
      cancel_requested_.store(true);
      RCLCPP_WARN(get_logger(), "Cancel requested for current task.");
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
      bool success = false;
      std::string message = "";

      RCLCPP_INFO(get_logger(), "Executing %s (step_id=%u)...", step.action.c_str(), step_id);

      // --- LOGICA DE ACCIONES ---
      
      if (step.action == "goto") 
      {
          std::string location = step.args[0];
          publish_feedback(goal_handle, step_id, "Moving to " + location);
          
          success = nav_->GoTo(location); 
          message = success ? ("Arrived to " + location) : ("Failed goto " + location);
      }
      else if (step.action == "speak")
      {
          std::string text_to_say = step.args[0];
          publish_feedback(goal_handle, step_id, "Speaking: " + text_to_say);

          // Usamos hri_ -> Say()
          success = hri_->Say(text_to_say);
          
          message = success ? "Finished speaking" : "Failed to speak";
      }
      else if (step.action == "listen")
      {
          // Si el usuario no mandó argumentos, mandamos gramática vacía ""
          std::string grammar = step.args.empty() ? "" : step.args[0];
          
          publish_feedback(goal_handle, step_id, "Listening (grammar: " + grammar + ")...");

          // Usamos hri_ -> Listen()
          std::string heard_text = hri_->Listen(grammar);
          
          // Asumimos éxito si escuchó ALGO (string no vacio)
          if (!heard_text.empty()) {
              success = true;
              message = "Heard: " + heard_text;
              RCLCPP_INFO(get_logger(), "HRI Heard: %s", heard_text.c_str());
          } else {
              success = false;
              message = "Listen timed out or failed";
          }
      }

      // --- Verificación de Cancelación ---
      if (goal_handle->is_canceling() || cancel_requested_.load()) {
        auto result = std::make_shared<ExecuteTask::Result>();
        result->result.step_id = step_id;
        result->result.status = executor_interfaces::msg::TaskResult::ABORTED;
        result->result.message = "Canceled: " + step.action;

        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Task canceled (step_id=%u).", step_id);
        return;
      }

      // --- Resultado Final ---
      auto result = std::make_shared<ExecuteTask::Result>();
      result->result.step_id = step_id;
      result->result.message = message;

      if (success) {
        result->result.status = executor_interfaces::msg::TaskResult::SUCCESS;
        goal_handle->succeed(result);
      } else {
        result->result.status = executor_interfaces::msg::TaskResult::FAILURE;
        goal_handle->abort(result);
      }
    }

    void publish_feedback(const std::shared_ptr<GoalHandleExecuteTask>& goal_handle, 
                          uint32_t step_id, const std::string& msg)
    {
      auto fb = std::make_shared<ExecuteTask::Feedback>();
      fb->feedback.step_id = step_id;
      fb->feedback.progress = 0.5f; 
      fb->feedback.message = msg;
      goal_handle->publish_feedback(fb);
    }

  private:
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

    std::shared_ptr<Navigation> nav_;
    std::shared_ptr<HRI> hri_;
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