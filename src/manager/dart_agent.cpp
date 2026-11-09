#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"
#include "manager/runtime/task_builder.hpp"
#include "manager/tasks/task_registry.hpp"

#include <rmcs_dart_guidance/resource/new_launcher_resource.hpp>
#include <rmcs_dart_guidance/settings/dart_settings.hpp>

#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgent
//   新镖架的上位机任务代理，也是全部决策的所在地。
//
//   职责：
//     ① 收命令字符串 → 查 tasks::kAll 表 → 造任务进队列
//     ② 每 tick 推进队首任务（串行）
//     ③ 直接把电机目标写进下位机 controller（速度/力矩/标零）
//     ④ 读回编码器与力矩，由动作自己判断堵转与到位
//
//   任务的「有哪些」「什么顺序」全在 tasks/ 目录里，加任务不改本文件一行。
// ═════════════════════════════════════════════════════════════════════════════
class DartAgent
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAgent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
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

        load_settings();

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
        // 拨杆只用于急停判断，缺了就当「不在双下」，不影响任务运行
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/remote/switch/left\". E-STOP disabled.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(
                logger_, "Failed to fetch \"/remote/switch/right\". E-STOP disabled.");
        }
    }

    // 注意顺序：急停检查 → 收命令 → 推进任务 → 发布状态。
    // 急停必须在最前，保证任何一条 tick 都不会在失能状态下继续写目标值。
    void update() override {
        if (check_emergency_stop())
            return;  // 急停期间不推进任务，只发布状态
        poll_commands();
        step_queue();
        publish_state();
    }

private:
    // 只用来挂 output 接口的哑组件
    class CommandComponent : public rmcs_executor::Component {
    public:
        void update() override {}
    };

    // ── ⓪ 急停：双下 ────────────────────────────────────────────────────────
    //
    // 硬件层（dart_new_launcher.cpp:update_motor_enable）已经把「双下」定义成
    // 全电机 NaN 失能。但硬件层只是不再执行力矩，它不知道上位机还有任务在跑。
    // 如果上位机不停，队列会继续推进，动作会一直以为自己在前述中——
    // 一旦拨杆离开双下，硬件层恢复执行，机构会突然执行积压的、早就过时的目标。
    //
    // 所以上位机必须自己响应：清空队列 + 机构置为保持（零速 + 力矩限幅）。
    // 注意不是发 NaN —— 上位机这一层永远发有限值，NaN 只由硬件层决定。
    //
    // 返回 true 表示本 tick 处于急停态，update() 应当跳过任务推进。
    bool check_emergency_stop() {
        const bool both_down =
            switch_left_.ready() && switch_right_.ready()
            && *switch_left_ == rmcs_msgs::Switch::DOWN
            && *switch_right_ == rmcs_msgs::Switch::DOWN;

        if (!both_down) {
            if (estop_latched_) {
                estop_latched_ = false;
                RCLCPP_INFO(
                    logger_, "[DartAgent] E-STOP released, queue is empty, awaiting command");
            }
            return false;
        }

        if (!estop_latched_) {
            estop_latched_ = true;
            const std::size_t dropped = queue_.size();
            const bool had_work = dropped > 0 || busy_;

            halt_mechanisms();
            queue_.clear();
            task_started_ = false;
            busy_ = false;
            current_task_.clear();

            if (had_work) {
                last_error_ = "E-STOP: both switches DOWN";
                RCLCPP_ERROR(
                    logger_, "[DartAgent] E-STOP: both switches DOWN, dropped %zu task(s)", dropped);
            } else {
                RCLCPP_WARN(logger_, "[DartAgent] E-STOP: both switches DOWN");
            }
        }

        // 急停期间也要发布状态，否则外部看不到 bus 已经落下
        publish_state();
        return true;
    }

    // ── ① 收命令 ────────────────────────────────────────────────────────────
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

    // ── ② 推进队列 ──────────────────────────────────────────────────────────
    // 串行：队首跑完才起下一个。失败则清空整队并停机构，不做自动重试。
    void step_queue() {
        if (queue_.empty()) {
            busy_ = false;
            current_task_.clear();
            return;
        }

        busy_ = true;
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

            halt_mechanisms();
            queue_.clear();
            task_started_ = false;
            busy_ = false;
            current_task_.clear();
        }
    }

    // 任务失败时把两个机构都停住，不留残余运动
    void halt_mechanisms() {
        belt_resource_.hold(settings_.homing_torque_limit);
        carriage_resource_.hold();
    }

    void publish_state() {
        *fire_count_output_ = state_.fire_count;
        *current_task_output_ = current_task_;
        *busy_output_ = busy_;
        *last_error_output_ = last_error_;
    }

    // ── 参数 ────────────────────────────────────────────────────────────────
    void load_settings() {
        const auto d = [this](const char* n) {
            return has_parameter(n) ? get_parameter(n).as_double() : 0.0;
        };

        settings_.dart_count = static_cast<int>(get_parameter("dart_count").as_int());
        settings_.belt_trigger_distance = d("belt_trigger_distance");

        settings_.belt_home_velocity = d("belt_home_velocity");
        settings_.belt_down_velocity = d("belt_down_velocity");
        settings_.belt_up_velocity = d("belt_up_velocity");
        settings_.homing_torque_limit = d("homing_torque_limit");
        settings_.belt_load_torque_limit = d("belt_load_torque_limit");
        settings_.belt_unload_torque_limit = d("belt_unload_torque_limit");

        settings_.carriage_home_velocity = d("carriage_home_velocity");
        settings_.carriage_back_off_velocity = d("carriage_back_off_velocity");
        settings_.carriage_back_off_distance = d("carriage_back_off_distance");
        settings_.carriage_goto_velocity = d("carriage_goto_velocity");
        settings_.carriage_goto_gain = d("carriage_goto_gain");
        settings_.carriage_goto_tolerance = d("carriage_goto_tolerance");

        for (int i = 0; i < DartSettings::kMaxSlots; ++i) {
            const auto name = "dart_" + std::to_string(i) + "_carriage_staypoint";
            if (has_parameter(name))
                settings_.dart_carriage_staypoints[static_cast<std::size_t>(i)] =
                    get_parameter(name).as_double();
        }
    }

    // ── 成员 ────────────────────────────────────────────────────────────────
    rclcpp::Logger logger_;

    std::shared_ptr<CommandComponent> command_component_;
    NewBeltResource belt_resource_;
    NewCarriageResource carriage_resource_;

    DartSettings settings_;
    manager::ManagerRuntimeState state_;
    TaskContext context_;

    InputInterface<std::string> command_input_;
    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<uint32_t> fire_count_output_;
    OutputInterface<std::string> current_task_output_;
    OutputInterface<bool> busy_output_;
    OutputInterface<std::string> last_error_output_;

    std::deque<std::shared_ptr<manager::Task>> queue_;
    bool task_started_{false};
    bool busy_{false};
    std::string current_task_;
    std::string last_command_;
    std::string last_error_;

    // 急停锁存：只在进入双下那一刻清一次队列，避免每 tick 重复触发
    bool estop_latched_{false};
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::v2::DartAgent, rmcs_executor::Component)
