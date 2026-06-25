#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "executor_interfaces/action/execute_task.hpp"
#include "executor_interfaces/msg/task_result.hpp"

#include "executor_tools/Align.hpp"
#include "executor_tools/Approach.hpp"
#include "executor_tools/Check.hpp"
#include "executor_tools/Drop.hpp"
#include "executor_tools/FindSpace.hpp"
#include "executor_tools/FindObject.hpp"
#include "executor_tools/FindPerson.hpp"
#include "executor_tools/FindPose.hpp"
#include "executor_tools/Focus.hpp"
#include "executor_tools/Follow.hpp"
#include "executor_tools/Give.hpp"
#include "executor_tools/GoTo.hpp"
#include "executor_tools/Listen.hpp"
#include "executor_tools/Point.hpp"
#include "executor_tools/Receive.hpp"
#include "executor_tools/Result.hpp"
#include "executor_tools/Say.hpp"
#include "executor_tools/Take.hpp"
#include "executor_tools/Analyze.hpp"
#include "executor_tools/AnalyzePerson.hpp"
#include "executor_tools/SaveLoc.hpp"
#include "executor_tools/Fold.hpp"

#include <optional>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "hardware_tools/Hardware.hpp"
#include "hri_tools/HRI.hpp"
#include "knowledge_tools/Knowledge.hpp"
#include "manipulation_tools/Manipulation.hpp"
#include "navigation_tools/Navigation.hpp"
#include "planning_tools/Planning.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ExecutorWorkerNode : public rclcpp::Node {
public:
  using ExecuteTask = executor_interfaces::action::ExecuteTask;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ExecuteTask>;
  using TaskResult = executor_interfaces::msg::TaskResult;
  using ActionFunction = std::function<Result(
      const std::string &, const std::shared_ptr<GoalHandle> &)>;

  ExecutorWorkerNode() : Node("executor_worker_node") {
    action_server_ = rclcpp_action::create_server<ExecuteTask>(
        this, "/executor/execute_task",
        std::bind(&ExecutorWorkerNode::handleGoal, this, _1, _2),
        std::bind(&ExecutorWorkerNode::handleCancel, this, _1),
        std::bind(&ExecutorWorkerNode::handleAccepted, this, _1));

    RCLCPP_INFO(get_logger(), "ExecutorWorkerNode ready.");
  }

  void initTools() {
    auto node = shared_from_this();

    planning_ = std::make_shared<Planning>(node);
    knowledge_ = std::make_shared<Knowledge>(node);

    hri_ = std::make_shared<HRI>(node);
    say_ = std::make_shared<Say>(node, hri_, knowledge_);
    listen_ = std::make_shared<Listen>(node, hri_, planning_, knowledge_);

    customer_pose_map_ = std::make_shared<std::optional<geometry_msgs::msg::PoseStamped>>(std::nullopt);

    hardware_ = std::make_shared<Hardware>(node);
    check_ = std::make_shared<Check>(node, hardware_);
    point_ = std::make_shared<Point>(node, hardware_);
    give_ = std::make_shared<Give>(node, hardware_, knowledge_, hri_);
    receive_ = std::make_shared<Receive>(node, hardware_, hri_, knowledge_);
    follow_ = std::make_shared<Follow>(node);

    navigation_ = std::make_shared<Navigation>(node);
    go_to_ = std::make_shared<GoTo>(node, hardware_, navigation_);
    save_loc_ = std::make_shared<SaveLoc>(node, navigation_, hardware_, customer_pose_map_);

    vision_ = std::make_shared<Vision>(node);
    find_person_ = std::make_shared<FindPerson>(node, hardware_, vision_, knowledge_);
    find_pose_ = std::make_shared<FindPose>(node, hardware_, hri_, knowledge_, customer_pose_map_);
    align_ = std::make_shared<Align>(node, hardware_, vision_);
    find_space_ = std::make_shared<FindSpace>(node, hardware_, knowledge_);
    manipulation_ = std::make_shared<Manipulation>(node);
    drop_ = std::make_shared<Drop>(node, manipulation_, hardware_, knowledge_, hri_);
    find_object_ = std::make_shared<FindObject>(node, hardware_, vision_, knowledge_);
    focus_ = std::make_shared<Focus>(node, vision_);
    take_ = std::make_shared<Take>(node, manipulation_, hardware_, knowledge_, hri_);
    analyze_objects_ = std::make_shared<AnalyzeObjects>(node, hardware_, vision_, knowledge_);
    analyze_person_ = std::make_shared<AnalyzePerson>(node, hardware_, vision_, knowledge_);

	fold_ = std::make_shared<Fold>(node, hardware_, vision_);

    approach_ = std::make_shared<Approach>(node, hardware_, vision_, knowledge_, navigation_);

    actionMap_["say"] = [this](const std::string &arg,
                               const std::shared_ptr<GoalHandle> &gh) {
      return say_->run(arg, gh);
    };

    actionMap_["listen"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return listen_->run(arg, gh);
    };

    actionMap_["go_to"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return go_to_->run(arg, gh);
    };

    actionMap_["check"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return check_->run(arg, gh);
    };

    actionMap_["point"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return point_->run(arg, gh);
    };

    actionMap_["give"] = [this](const std::string &arg,
                                const std::shared_ptr<GoalHandle> &gh) {
      return give_->run(arg, gh);
    };

    actionMap_["find_person"] = [this](const std::string &arg,
                                       const std::shared_ptr<GoalHandle> &gh) {
      return find_person_->run(arg, gh);
    };

    actionMap_["align"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return align_->run(arg, gh);
    };

    actionMap_["drop"] = [this](const std::string &arg,
                                const std::shared_ptr<GoalHandle> &gh) {
      return drop_->run(arg, gh);
    };

    actionMap_["find_space"] = [this](const std::string &arg,
                                      const std::shared_ptr<GoalHandle> &gh) {
      return find_space_->run(arg, gh);
    };

    actionMap_["find_object"] = [this](const std::string &arg,
                                       const std::shared_ptr<GoalHandle> &gh) {
      return find_object_->run(arg, gh);
    };

    actionMap_["focus"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return focus_->run(arg, gh);
    };

    actionMap_["receive"] = [this](const std::string &arg,
                                   const std::shared_ptr<GoalHandle> &gh) {
      return receive_->run(arg, gh);
    };
    actionMap_["follow"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return follow_->run(arg, gh);
    };

    actionMap_["approach"] = [this](const std::string &arg,
                                    const std::shared_ptr<GoalHandle> &gh) {
      return approach_->run(arg, gh);
    };

    actionMap_["approach_partial"] = [this](const std::string &arg,
                                            const std::shared_ptr<GoalHandle> &gh) {
      return approach_->runPartial(arg, gh);
    };

    actionMap_["take"] = [this](const std::string &arg,
                                const std::shared_ptr<GoalHandle> &gh) {
      return take_->run(arg, gh);
    };

    actionMap_["find_pose"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return find_pose_->run(arg, gh);
    };

    actionMap_["analyze_objects"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return analyze_objects_->run(arg, gh);
    };

    actionMap_["analyze_person"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return analyze_person_->run(arg, gh);
    };

    actionMap_["save_loc"] = [this](const std::string &arg,
                                   const std::shared_ptr<GoalHandle> &gh) {
      return save_loc_->run(arg, gh);
    };
	
	actionMap_["fold"] = [this](const std::string &arg,
                                  const std::shared_ptr<GoalHandle> &gh) {
      return fold_->run(arg, gh);
    };
  }

private:
  // =========================================================
  // Action callbacks
  // =========================================================
  rclcpp_action::GoalResponse
  handleGoal(const rclcpp_action::GoalUUID &,
             std::shared_ptr<const ExecuteTask::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handleCancel(const std::shared_ptr<GoalHandle>) {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> gh) {
    std::thread(&ExecutorWorkerNode::execute, this, gh).detach();
  }

  // =========================================================
  // Execution
  // =========================================================
  void execute(const std::shared_ptr<GoalHandle> gh) {
    if (!gh) {
      RCLCPP_ERROR(get_logger(), "Null goalHandle — aborting.");
      return;
    }

    const auto &step = gh->get_goal()->task;
    const std::string arg = step.args.empty() ? "" : step.args[0];

    Result tr{false, "FAILED", "unknown_action: " + step.action};

    try {
      auto it = actionMap_.find(step.action);

      if (it != actionMap_.end()) {
        RCLCPP_INFO(get_logger(), "Action: %s  arg: %s", step.action.c_str(),
                    arg.c_str());
        tr = it->second(arg, gh);
      } else {
        RCLCPP_WARN(get_logger(), "Unknown action: %s", step.action.c_str());
        tr = {false, "FAILED", "unknown_action: " + step.action};
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Tool '%s' threw: %s", step.action.c_str(),
                   e.what());
      tr = {false, "FAILED", std::string("exception: ") + e.what()};
    } catch (...) {
      RCLCPP_ERROR(get_logger(), "Tool '%s' caused unknown error.",
                   step.action.c_str());
      tr = {false, "FAILED", "unknown_error"};
    }

    auto result = std::make_shared<ExecuteTask::Result>();
    result->result.step_id = step.step_id;
    result->result.message = tr.message;
    result->result.status =
        tr.success ? TaskResult::SUCCESS : TaskResult::FAILURE;

    if (!gh->is_active()) {
      RCLCPP_WARN(get_logger(),
                  "GoalHandle no longer active — skipping result.");
      return;
    }

    if (tr.success)
      gh->succeed(result);
    else
      gh->abort(result);
  }

  // =========================================================
  // Members
  // =========================================================
  rclcpp_action::Server<ExecuteTask>::SharedPtr action_server_;

  std::shared_ptr<Planning> planning_;
  std::shared_ptr<Knowledge> knowledge_;
  std::shared_ptr<HRI> hri_;
  std::shared_ptr<Say> say_;
  std::shared_ptr<Listen> listen_;

  std::shared_ptr<Navigation> navigation_;
  std::shared_ptr<GoTo> go_to_;
  std::shared_ptr<SaveLoc> save_loc_;
  std::shared_ptr<std::optional<geometry_msgs::msg::PoseStamped>> customer_pose_map_;

  std::shared_ptr<Hardware> hardware_;
  std::shared_ptr<Check> check_;
  std::shared_ptr<Point> point_;
  std::shared_ptr<Give> give_;
  std::shared_ptr<Receive> receive_;
  std::shared_ptr<Follow> follow_;

  std::shared_ptr<Vision> vision_;
  std::shared_ptr<FindPerson> find_person_;
  std::shared_ptr<FindPose> find_pose_;
  std::shared_ptr<Align> align_;  
  std::shared_ptr<FindSpace> find_space_;
  std::shared_ptr<Drop> drop_;
  std::shared_ptr<FindObject> find_object_;
  std::shared_ptr<Focus> focus_;
  std::shared_ptr<Manipulation> manipulation_;
  std::shared_ptr<Take> take_;
  std::shared_ptr<AnalyzeObjects> analyze_objects_;
  std::shared_ptr<AnalyzePerson> analyze_person_;

  std::shared_ptr<Fold> fold_;  

  std::shared_ptr<Approach> approach_;

  std::unordered_map<std::string, ActionFunction> actionMap_;
};

// =========================================================
// Main
// =========================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecutorWorkerNode>();
  node->initTools();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
