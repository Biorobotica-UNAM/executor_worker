#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/msg/task_result.hpp"

#include "executor_tools/Result.hpp"
#include "executor_tools/Say.hpp"
#include "executor_tools/GoTo.hpp"
#include "executor_tools/Check.hpp"
#include "executor_tools/Point.hpp"

#include "hri_tools/HRI.hpp"
#include "navigation_tools/Navigation.hpp"
#include "hardware_tools/Hardware.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ExecutorWorkerNode : public rclcpp::Node
{
public:
    using ExecuteTask    = executor_interfaces::action::ExecuteTask;
    using GoalHandle     = rclcpp_action::ServerGoalHandle<ExecuteTask>;
    using TaskResult     = executor_interfaces::msg::TaskResult;
    using ActionFunction = std::function<Result(
        const std::string&, const std::shared_ptr<GoalHandle>&)>;

    ExecutorWorkerNode() : Node("executor_worker_node")
    {
        action_server_ = rclcpp_action::create_server<ExecuteTask>(
            this, "/executor/execute_task",
            std::bind(&ExecutorWorkerNode::handleGoal,     this, _1, _2),
            std::bind(&ExecutorWorkerNode::handleCancel,   this, _1),
            std::bind(&ExecutorWorkerNode::handleAccepted, this, _1));

        RCLCPP_INFO(get_logger(), "ExecutorWorkerNode ready.");
    }

    void initTools()
    {
        auto node = shared_from_this();

        hri_ = std::make_shared<HRI>(node);
        say_ = std::make_shared<Say>(node, hri_);

        navigation_ = std::make_shared<Navigation>(node);
		go_to_ = std::make_shared<GoTo>(node, navigation_);

		hardware_ = std::make_shared<Hardware>(node);
		check_ = std::make_shared<Check>(node, hardware_);
		point_ = std::make_shared<Point>(node, hardware_);
		

        actionMap_["say"] =
            [this](const std::string& arg, const std::shared_ptr<GoalHandle>& gh) {
                return say_->run(arg, gh);
            };

		actionMap_["go_to"] =
            [this](const std::string& arg, const std::shared_ptr<GoalHandle>& gh) {
                return go_to_->run(arg, gh);
            };

		actionMap_["check"] =
            [this](const std::string& arg, const std::shared_ptr<GoalHandle>& gh) {
                return check_->run(arg, gh);
            };

		actionMap_["point"] =
            [this](const std::string& arg, const std::shared_ptr<GoalHandle>& gh) {
                return point_->run(arg, gh);
            };
    }

private:

    // =========================================================
    // Action callbacks
    // =========================================================
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const ExecuteTask::Goal>)
    {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandle>)
    {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandle> gh)
    {
        std::thread(&ExecutorWorkerNode::execute, this, gh).detach();
    }

    // =========================================================
    // Execution
    // =========================================================
    void execute(const std::shared_ptr<GoalHandle> gh)
    {
        if (!gh) {
            RCLCPP_ERROR(get_logger(), "Null goalHandle — aborting.");
            return;
        }

        const auto& step     = gh->get_goal()->task;
        const std::string arg = step.args.empty() ? "" : step.args[0];

        Result tr{false, "FAILED", "unknown_action: " + step.action};

        try {
            auto it = actionMap_.find(step.action);

            if (it != actionMap_.end()) {
                RCLCPP_INFO(get_logger(), "Action: %s  arg: %s",
                            step.action.c_str(), arg.c_str());
                tr = it->second(arg, gh);
            } else {
                RCLCPP_WARN(get_logger(), "Unknown action: %s", step.action.c_str());
                tr = {false, "FAILED", "unknown_action: " + step.action};
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(get_logger(), "Tool '%s' threw: %s",
                         step.action.c_str(), e.what());
            tr = {false, "FAILED", std::string("exception: ") + e.what()};
        }
        catch (...) {
            RCLCPP_ERROR(get_logger(), "Tool '%s' caused unknown error.",
                         step.action.c_str());
            tr = {false, "FAILED", "unknown_error"};
        }

        auto result          = std::make_shared<ExecuteTask::Result>();
        result->result.step_id = step.step_id;
        result->result.message = tr.message;
        result->result.status  = tr.success
            ? TaskResult::SUCCESS
            : TaskResult::FAILURE;

        if (!gh->is_active()) {
            RCLCPP_WARN(get_logger(), "GoalHandle no longer active — skipping result.");
            return;
        }

        if (tr.success) gh->succeed(result);
        else            gh->abort(result);
    }

    // =========================================================
    // Members
    // =========================================================
    rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

    std::shared_ptr<HRI> hri_;
    std::shared_ptr<Say> say_;

    std::shared_ptr<Navigation> navigation_;
	std::shared_ptr<GoTo> go_to_;

    std::shared_ptr<Hardware> hardware_;
	std::shared_ptr<Check> check_;
	std::shared_ptr<Point> point_;

    std::unordered_map<std::string, ActionFunction> actionMap_;
};

// =========================================================
// Main
// =========================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ExecutorWorkerNode>();
    node->initTools();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
