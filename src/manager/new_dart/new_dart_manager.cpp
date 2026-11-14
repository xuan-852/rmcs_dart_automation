#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include <rmcs_dart_guidance/new_dart/new_dart_task_builder.hpp>
#include <rmcs_dart_guidance/new_dart/new_dart_task_registry.hpp>

#include <rmcs_dart_guidance/new_dart/new_dart_state.hpp>
#include <rmcs_dart_guidance/new_dart/new_dart_io.hpp>
#include <rmcs_dart_guidance/new_dart/new_dart_settings.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgent
//   新镖架的上位机任务代理。
//
//   职责：
//     ① 收命令字符串 → 查 tasks::kAll 表 → 造任务进队列
//     ② 每 tick 推进队首任务（串行）
//     ③ 直接把电机目标写进下位机 controller（速度/力矩/标零）
//     ④ 读回编码器与力矩，由动作自己判断堵转与到位
//     ⑤ 双下急停时清队列 + 停机构
// ═════════════════════════════════════════════════════════════════════════════
class DartAgent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAgent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger())
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command"))
        , belt_resource_(*this, *command_component_, "/dart/belt")
        , carriage_resource_(*this, *command_component_, "/dart/carriage_motor")
        , context_{belt_resource_, carriage_resource_, settings_, state_} {

        register_input("/dart/manager/command", command_input_, false);
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);
        register_output("/dart/manager/fire_count", fire_count_output_, uint32_t{0});
        register_output("/dart/agent/current_task", current_task_output_, std::string{});
        register_output("/dart/agent/busy", busy_output_, false);
        register_output("/dart/agent/last_error", last_error_output_, std::string{});

        ops_ = AgentOps{
            this,
            &DartAgent::ops_transition_to,
            &DartAgent::ops_enter_idle,
            &DartAgent::ops_pump_tasks,
            &DartAgent::ops_abort_all,
        };

        // ── 参数加载（构造期一次性，表驱动：加参数改 new_dart_settings.hpp）──
        for (const auto& item : kIntSettings) {
            if (has_parameter(item.name))
                settings_.*(item.field) = static_cast<int>(get_parameter(item.name).as_int());
        }
        for (const auto& item : kDoubleSettings) {
            if (has_parameter(item.name))
                settings_.*(item.field) = get_parameter(item.name).as_double();
        }
        // 停位参数名带索引，进不了表，单独循环
        for (int i = 0; i < DartSettings::kMaxSlots; ++i) {
            const auto name = "dart_" + std::to_string(i) + "_carriage_staypoint";
            if (has_parameter(name))
                settings_.dart_carriage_staypoints[static_cast<std::size_t>(i)] =
                    get_parameter(name).as_double();
        }

        // ── 启动自检 ────────────────────────────────────────────────────────
        RCLCPP_INFO(
            logger_, "[DartAgent] %zu tasks registered, dart_count=%d",
            sizeof(tasks::kAll) / sizeof(tasks::kAll[0]), settings_.dart_count);
        if (!settings_.configured()) {
            RCLCPP_WARN(
                logger_,
                "[DartAgent] settings incomplete (dart_count=%d, belt_trigger_distance=%.3f). "
                "Tasks needing them will fail.",
                settings_.dart_count, settings_.belt_trigger_distance);
        }
    }

    DartAgent(const DartAgent&) = delete;
    DartAgent& operator=(const DartAgent&) = delete;
    DartAgent(DartAgent&&) = delete;
    DartAgent& operator=(DartAgent&&) = delete;

    ~DartAgent() override = default;

    void before_updating() override {
        belt_resource_.bind_optional(logger_);
        carriage_resource_.bind_optional(logger_);

        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(std::string{});
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/dart/manager/command\". Set to empty string.");
        }
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/left\". E-STOP disabled.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/right\". E-STOP disabled.");
        }
    }

    // 一行分派到当前状态的 tick，状态行为见 dart_agent_state.cpp
    void update() override {
        agent_state_entry(agent_state_).tick(ops_, is_both_down());
        publish_state();
    }

private:
    // 只用来挂 output 接口的哑组件
    class CommandComponent : public rmcs_executor::Component {
    public:
        void update() override {}
    };

    // ── 状态转换 ────────────────────────────────────────────────────────────
    void transition_to(AgentState next) {
        if (next == agent_state_)
            return;

        RCLCPP_INFO(
            logger_, "[DartAgent] %s -> %s", agent_state_entry(agent_state_).name,
            agent_state_entry(next).name);

        agent_state_ = next;
        agent_state_entry(agent_state_).enter(ops_);
    }

    bool is_both_down() const {
        return switch_left_.ready() && switch_right_.ready()
            && *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::DOWN;
    }

    // ── AgentOps 转发：状态机只认函数指针，这里转给成员函数 ──────────────────
    static void ops_transition_to(void* ctx, AgentState next) {
        static_cast<DartAgent*>(ctx)->transition_to(next);
    }
    static void ops_enter_idle(void* ctx) {
        static_cast<DartAgent*>(ctx)->current_task_.clear();
    }
    static bool ops_pump_tasks(void* ctx) {
        return static_cast<DartAgent*>(ctx)->pump_tasks();
    }
    static void ops_abort_all(void* ctx, const char* reason) {
        static_cast<DartAgent*>(ctx)->abort_all(reason);
    }

    // ── 任务推进：收命令 + 跑队首 ────────────────────────────────────────────
    // 返回 true 表示当前有任务在跑（供状态机判断是否离开 IDLE）。
    bool pump_tasks() {
        poll_commands();
        step_queue();
        return !queue_.empty();
    }

    // ── 急停 / 失败：停机构 + 清队列 + 报错 ──────────────────────────────────
    // reason 为 nullptr 时只停不报（任务失败时由 step_queue 自己打过日志）。
    void abort_all(const char* reason) {
        const std::size_t dropped = queue_.size();

        // 停机构：零速 + 力矩保持（不用 NaN，竖直滑台会自由下滑）
        belt_resource_.hold(settings_.homing_torque_limit);
        carriage_resource_.hold();

        // 清队列
        queue_.clear();
        task_started_ = false;
        current_task_.clear();

        if (reason == nullptr)
            return;

        last_error_ = reason;
        if (dropped > 0) {
            RCLCPP_ERROR(
                logger_, "[DartAgent] %s, dropped %zu task(s)", reason, dropped);
        } else {
            RCLCPP_WARN(logger_, "[DartAgent] %s", reason);
        }
    }

    // ── 收命令 ──────────────────────────────────────────────────────────────
    // 命令是电平信号：同一字符串连续多帧只接受一次，靠 last_command_ 去重。
    void poll_commands() {
        const std::string command = command_input_.ready() ? *command_input_ : std::string{};

        if (command.empty()) {
            last_command_.clear();
            return;
        }
        if (command == last_command_)
            return;
        last_command_ = command;

        const tasks::TaskEntry* entry = tasks::find(command);
        if (entry == nullptr) {
            last_error_ = "unknown command: " + command;
            RCLCPP_ERROR(logger_, "[DartAgent] unknown command \"%s\"", command.c_str());
            return;
        }

        queue_.push_back(entry->factory(context_));
        RCLCPP_INFO(
            logger_, "[DartAgent] queued \"%s\" (%s)", std::string(entry->name).c_str(),
            std::string(entry->description).c_str());
    }

    // ── 推进队列 ────────────────────────────────────────────────────────────
    // 串行：队首跑完才起下一个。失败则进入 kFailed 并停机构，不做自动重试。
    void step_queue() {
        if (queue_.empty()) {
            transition_to(AgentState::kIdle);
            return;
        }

        auto& task = queue_.front();
        current_task_ = task->name();

        if (!task_started_) {
            task->bind_runtime_context({task->name(), &logger_});
            task->tick_first();
            task_started_ = true;
            return;
        }

        const manager::ActionStatus status = task->tick();

        if (status == manager::ActionStatus::SUCCESS) {
            task->finish_success();
            RCLCPP_INFO(logger_, "[DartAgent] \"%s\" succeeded", task->name().c_str());
            queue_.pop_front();
            task_started_ = false;
            return;
        }

        if (status == manager::ActionStatus::FAILURE) {
            task->finish_failure();
            last_error_ = "task \"" + task->name() + "\" failed at \""
                        + task->failure_info().action_name + "\"";
            RCLCPP_ERROR(logger_, "[DartAgent] %s", last_error_.c_str());
            transition_to(AgentState::kFailed);   // enter 里会调 abort_all
        }
    }

    // ── 状态发布 ────────────────────────────────────────────────────────────
    void publish_state() {
        *fire_count_output_ = state_.fire_count;
        *current_task_output_ = current_task_;
        *busy_output_ = (agent_state_ == AgentState::kRunning);
        *last_error_output_ = last_error_;
    }

    // ── 成员 ────────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;
    std::shared_ptr<CommandComponent> command_component_;
    NewBeltResource belt_resource_;
    NewCarriageResource carriage_resource_;
    DartSettings settings_;
    manager::ManagerRuntimeState state_;
    TaskContext context_;

    // 状态机
    AgentOps ops_{};
    AgentState agent_state_{AgentState::kIdle};

    // 输入
    InputInterface<std::string> command_input_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    // 输出
    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> current_task_output_;
    OutputInterface<bool> busy_output_;
    OutputInterface<std::string> last_error_output_;

    // 任务队列
    std::deque<std::shared_ptr<manager::Task>> queue_;
    bool task_started_{false};
    std::string current_task_;
    std::string last_command_;
    std::string last_error_;
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::v2::DartAgent, rmcs_executor::Component)

