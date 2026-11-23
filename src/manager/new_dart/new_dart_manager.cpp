#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/new_dart/new_dart_tasks.hpp>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgent
//   复用老镖架 manager/runtime 的任务队列、ERROR 锁存与 recover 模型；
//   中间通过 BeltActionResource / CarriageActionResource 转接新镖架执行组件。
// ═════════════════════════════════════════════════════════════════════════════
class DartAgent
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using ActionCancelReason = manager::ActionCancelReason;
    using ActionRuntimeContext = manager::ActionRuntimeContext;
    using ActionStatus = manager::ActionStatus;
    using LifecycleState = manager::ManagerLifecycleState;

public:
    DartAgent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command"))
        , belt_(*this, *command_component_, "/dart/belt_action")
        , carriage_(*this, *command_component_, "/dart/carriage_action") {
            
        // ── 1. 注册输入与状态输出 ───────────────────────────────────────────
        register_input("/dart/manager/command", command_input_, false);

        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        register_output("/dart/agent/current_task", current_task_output_, std::string{});
        register_output("/dart/agent/busy", busy_output_, false);
        register_output("/dart/agent/last_error", last_error_output_, std::string{});

        // ── 2. 读取参数 ─────────────────────────────────────────────────────
        int64_t dart_count = 0;
        get_parameter_or("dart_count", dart_count, int64_t{0});
        settings_.dart_count = static_cast<int>(dart_count);
        for (int i = 0; i < NewDartSettings::kMaxSlots; ++i) {
            get_parameter_or(
                "dart_" + std::to_string(i) + "_carriage_staypoint",
                settings_.staypoints[static_cast<std::size_t>(i)], 0.0);
        }
        get_parameter_or("belt_trigger_distance", settings_.belt_trigger_distance, 0.0);

        RCLCPP_INFO(
            logger_, "[DartAgent] initialized (dart_count=%d, trigger_dist=%.3f)",
            settings_.dart_count, settings_.belt_trigger_distance);
    }

    DartAgent(const DartAgent&) = delete;
    DartAgent& operator=(const DartAgent&) = delete;
    DartAgent(DartAgent&&) = delete;
    DartAgent& operator=(DartAgent&&) = delete;

    ~DartAgent() override = default;

    void before_updating() override {
        belt_.bind_optional(logger_);
        carriage_.bind_optional(logger_);

        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(logger_, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
    }

    void update() override {
        poll_commands();

        while (runtime_state_.lifecycle_state != LifecycleState::ERROR) {
            if (!current_task_) {
                if (task_queue_.empty()) {
                    runtime_state_.lifecycle_state = LifecycleState::IDLE;
                    break;
                }
                dispatch_next_task();
            }
            if (!current_task_)
                break;

            if (tick_current_task() == ActionStatus::RUNNING
                || runtime_state_.lifecycle_state == LifecycleState::ERROR) {
                break;
            }
        }

        publish_state();
    }

private:
    class CommandComponent : public rmcs_executor::Component {
    public:
        void update() override {}
    };

    void poll_commands() {
        const std::string& command = *command_input_;
        if (command.empty()) {
            last_command_.clear();
            return;
        }
        if (command == last_command_)
            return;
        last_command_ = command;

        if (command == "cancel") {
            stop_tasks(LifecycleState::ERROR, "cancelled");
            return;
        }
        if (command == "recover") {
            recover();
            return;
        }

        if (runtime_state_.lifecycle_state == LifecycleState::ERROR) {
            RCLCPP_WARN(
                logger_, "[DartAgent] ignored \"%s\" while state=ERROR (send recover first)",
                command.c_str());
            return;
        }

        if (command == "stop") {//此处将接入手动控制，映射在底层conntroller的manual_active_
            if (runtime_state_.lifecycle_state == LifecycleState::RUNNING || !task_queue_.empty())
                stop_tasks(LifecycleState::IDLE, "stopped");
            else
                safe_stop_all();
            return;
        }

        auto task = make_task(command, belt_, carriage_, runtime_state_, settings_);
        if (!task) {
            last_error_ = "unknown command: " + command;
            RCLCPP_ERROR(logger_, "[DartAgent] unknown command \"%s\"", command.c_str());
            return;
        }

        RCLCPP_INFO(
            logger_, "[DartAgent] queued \"%s\" (%s)", task->name().c_str(),
            task->description().c_str());
        task_queue_.push_back(std::move(task));
    }

    void stop_tasks(LifecycleState next_state, const std::string& reason) {
        task_queue_.clear();
        if (current_task_) {
            current_task_->cancel(ActionCancelReason::EXTERNAL_CANCEL);
            current_task_.reset();
        }
        first_tick_of_task_ = true;
        safe_stop_all();
        last_error_ = reason;
        runtime_state_.lifecycle_state = next_state;

        if (next_state == LifecycleState::ERROR)
            RCLCPP_ERROR(logger_, "[DartAgent] ERROR: %s", reason.c_str());
        else
            RCLCPP_WARN(logger_, "[DartAgent] %s", reason.c_str());
    }

    void recover() {
        const bool was_error = (runtime_state_.lifecycle_state == LifecycleState::ERROR);
        task_queue_.clear();
        current_task_.reset();
        first_tick_of_task_ = true;
        runtime_state_.fire_count = 0;
        runtime_state_.lifecycle_state = LifecycleState::IDLE;
        last_error_.clear();
        safe_stop_all();

        if (auto init_task = make_task("dart-init", belt_, carriage_, runtime_state_, settings_)) {
            task_queue_.push_back(std::move(init_task));
            RCLCPP_INFO(
                logger_, "[DartAgent] %s -> queued \"dart-init\"",
                was_error ? "recovered from ERROR" : "re-init");
        }
    }

    void dispatch_next_task() {
        if (task_queue_.empty())
            return;

        current_task_ = std::move(task_queue_.front());
        task_queue_.pop_front();
        current_task_->bind_runtime_context(ActionRuntimeContext{current_task_->name(), &logger_});
        first_tick_of_task_ = true;
        runtime_state_.lifecycle_state = LifecycleState::RUNNING;
        RCLCPP_INFO(logger_, "[DartAgent] dispatching \"%s\"", current_task_->name().c_str());
    }

    ActionStatus tick_current_task() {
        const std::string task_name = current_task_->name();
        const ActionStatus status =
            first_tick_of_task_ ? current_task_->tick_first() : current_task_->tick();
        first_tick_of_task_ = false;

        if (status == ActionStatus::SUCCESS) {
            current_task_->finish_success();
            current_task_.reset();
            RCLCPP_INFO(logger_, "[DartAgent] \"%s\" SUCCESS", task_name.c_str());
            if (task_queue_.empty())
                runtime_state_.lifecycle_state = LifecycleState::IDLE;
            return status;
        }

        if (status == ActionStatus::FAILURE) {
            const auto failure = current_task_->failure_info();
            const std::string action =
                failure.action_name.empty() ? task_name : failure.action_name;
            current_task_->finish_failure();
            current_task_.reset();
            task_queue_.clear();
            safe_stop_all();
            last_error_ = task_name + " failed at " + action + " (" + manager::to_string(failure.reason) + ")";
            runtime_state_.lifecycle_state = LifecycleState::ERROR;
            RCLCPP_ERROR(logger_, "[DartAgent] %s -> state=ERROR", last_error_.c_str());
        }

        return status;
    }

    void safe_stop_all() {
        belt_.safe_stop();
        carriage_.safe_stop();
    }

    void publish_state() {
        *fire_count_output_ = runtime_state_.fire_count;
        *current_task_output_ = current_task_ ? current_task_->name() : std::string{};
        *busy_output_ = (runtime_state_.lifecycle_state == LifecycleState::RUNNING);
        *last_error_output_ = last_error_;
    }

    // ── 成员 ────────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;
    std::shared_ptr<CommandComponent> command_component_;
    BeltActionResource belt_;
    CarriageActionResource carriage_;

    NewDartSettings settings_;
    manager::ManagerRuntimeState runtime_state_;
    std::shared_ptr<manager::Task> current_task_;
    std::deque<std::shared_ptr<manager::Task>> task_queue_;
    bool first_tick_of_task_{true};

    std::string last_command_;
    std::string last_error_;

    // ── 输入接口 ────────────────────────────────────────────────────────────
    InputInterface<std::string> command_input_;

    // ── 输出接口 ────────────────────────────────────────────────────────────
    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> current_task_output_;
    OutputInterface<bool> busy_output_;
    OutputInterface<std::string> last_error_output_;
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::v2::DartAgent, rmcs_executor::Component)
