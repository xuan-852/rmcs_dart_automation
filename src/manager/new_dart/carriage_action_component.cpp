#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/new_dart_action.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// CarriageActionComponent
//   弹舱上层动作执行组件：接收 DartAgent 下发的 (seq, ActionType) 指令，
//   完成保持、撞开关归零、软件标零与相对零点的 P 控定位。
// ═════════════════════════════════════════════════════════════════════════════
class CarriageActionComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using ActionCmd = rmcs_dart_guidance::msg::CarriageActionCommand;
    using ActionType = rmcs_dart_guidance::msg::CarriageActionType;
    using Feedback = rmcs_dart_guidance::msg::ActionFeedback;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    CarriageActionComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // ── 1. 反馈输入 ─────────────────────────────────────────────────────
        register_input("/dart/carriage_action/command", command_input_, false);
        register_input("/dart/carriage_motor/angle", angle_, false);
        register_input("/dart/carriage_motor/velocity", velocity_, false);
        register_input("/dart/micro_switch", micro_switch_, false);

        // ── 2. 底层控制输出 ─────────────────────────────────────────────────
        register_output("/dart/carriage_motor/cmd_velocity", cmd_velocity_, 0.0);
        register_output("/dart/carriage_motor/calibrate_request", calibrate_, false);
        register_output("/dart/carriage_action/feedback", feedback_output_, Feedback{});

        // ── 3. 读取参数 ─────────────────────────────────────────────────────
        get_parameter_or("carriage_home_velocity", param_home_velocity_, 5.0);
        get_parameter_or("carriage_back_off_velocity", param_back_off_velocity_, 2.0);
        get_parameter_or("carriage_back_off_distance", param_back_off_distance_, 0.1);
        get_parameter_or("carriage_goto_velocity", param_goto_velocity_, 20.0);
        get_parameter_or("carriage_goto_gain", param_goto_gain_, 10.0);
        get_parameter_or("carriage_goto_tolerance", param_goto_tolerance_, 0.02);
    }

    CarriageActionComponent(const CarriageActionComponent&) = delete;
    CarriageActionComponent& operator=(const CarriageActionComponent&) = delete;
    CarriageActionComponent(CarriageActionComponent&&) = delete;
    CarriageActionComponent& operator=(CarriageActionComponent&&) = delete;

    ~CarriageActionComponent() override = default;

    void before_updating() override {
        bind_or(command_input_, ActionCmd{}, "/dart/carriage_action/command");
        bind_or(angle_, 0.0, "/dart/carriage_motor/angle");
        bind_or(velocity_, 0.0, "/dart/carriage_motor/velocity");
        bind_or(micro_switch_, false, "/dart/micro_switch");
    }

    void update() override {
        const ActionCmd cmd = *command_input_;
        const bool is_new = (cmd.seq != active_seq_);
        if (is_new) {
            active_seq_ = cmd.seq;
            active_type_ = cmd.type;
            active_ticks_ = 0;
        } else if (status_ == MechStatus::BUSY) {
            ++active_ticks_;
        }

        if (is_new || status_ == MechStatus::BUSY)
            status_ = dispatch_action(cmd, is_new);

        *feedback_output_ = Feedback{active_seq_, status_};
    }

private:
    enum class HomePhase { kSeekSwitch, kBackOff };

    template <typename T, typename V>
    void bind_or(InputInterface<T>& iface, V&& fallback, const char* name) {
        if (iface.ready())
            return;
        iface.make_and_bind_directly(T(std::forward<V>(fallback)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    MechStatus dispatch_action(const ActionCmd& cmd, bool is_new) {
        switch (active_type_) {
        case ActionType::IDLE:
            return MechStatus::IDLE;

        case ActionType::HOLD:
            drive_hold();
            return MechStatus::SUCCEEDED;

        case ActionType::HOME:
            return run_home(is_new);

        case ActionType::GOTO:
            return run_goto(cmd, is_new);

        case ActionType::CALIBRATE:
            return run_calibrate(is_new);
        }
        return MechStatus::IDLE;
    }

    // ── 动作：撞微动开关 → 退离指定角度 ───────────────────────────────────────
    MechStatus run_home(bool is_new) {
        if (is_new) {
            home_phase_ = HomePhase::kSeekSwitch;
            home_hit_angle_ = 0.0;
        }

        if (home_phase_ == HomePhase::kSeekSwitch) {
            drive_velocity(param_home_velocity_);
            if (*micro_switch_) {
                home_hit_angle_ = *angle_;
                home_phase_ = HomePhase::kBackOff;
                RCLCPP_INFO(
                    get_logger(), "[CarriageAction] switch hit at %.3f, backing off",
                    home_hit_angle_);
            } else if (active_ticks_ % 1000 == 0) {
                RCLCPP_INFO(
                    get_logger(), "[CarriageAction] seeking: angle=%.3f vel=%.3f", *angle_,
                    *velocity_);
            }
            return MechStatus::BUSY;
        }

        drive_velocity(param_back_off_velocity_);
        const double moved = std::abs(*angle_ - home_hit_angle_);
        if (moved >= std::abs(param_back_off_distance_)) {
            drive_hold();
            RCLCPP_INFO(get_logger(), "[CarriageAction] backed off %.3f, done", moved);
            return MechStatus::SUCCEEDED;
        }
        if (active_ticks_ % 1000 == 0) {
            RCLCPP_INFO(
                get_logger(), "[CarriageAction] backing off: %.4f / %.4f", moved,
                std::abs(param_back_off_distance_));
        }
        return MechStatus::BUSY;
    }

    // ── 动作：按软件虚拟零点定位到目标角度（P 控制）───────────────────────────
    MechStatus run_goto(const ActionCmd& cmd, bool is_new) {
        if (is_new) {
            if (!std::isfinite(cmd.target_angle)) {
                drive_hold();
                RCLCPP_ERROR(get_logger(), "[CarriageAction] invalid GOTO target_angle");
                return MechStatus::FAILED;
            }
            goto_target_rel_ = cmd.target_angle;
        }

        const double rel_angle = *angle_ - calib_zero_offset_;
        const double err = goto_target_rel_ - rel_angle;
        if (std::abs(err) <= std::abs(param_goto_tolerance_)) {
            drive_hold();
            return MechStatus::SUCCEEDED;
        }

        const double max_v = std::abs(param_goto_velocity_);
        drive_velocity(std::clamp(err * param_goto_gain_, -max_v, max_v));
        return MechStatus::BUSY;
    }

    // ── 动作：标零（记录软件虚拟零点，不修改底层硬件编码器）──────────────────
    MechStatus run_calibrate(bool is_new) {
        if (is_new)
            calib_zero_offset_ = *angle_;

        drive_hold();
        return active_ticks_ >= kCalibrateHoldTicks ? MechStatus::SUCCEEDED : MechStatus::BUSY;
    }

    // ── 底层输出收口 ────────────────────────────────────────────────────────
    void drive_velocity(double velocity) {
        *cmd_velocity_ = velocity;
        *calibrate_ = false;
    }

    void drive_hold() { drive_velocity(0.0); }

    static constexpr uint64_t kCalibrateHoldTicks = 5;

    // ── 参数 ────────────────────────────────────────────────────────────────
    double param_home_velocity_{5.0};
    double param_back_off_velocity_{2.0};
    double param_back_off_distance_{0.1};
    double param_goto_velocity_{20.0};
    double param_goto_gain_{10.0};
    double param_goto_tolerance_{0.02};

    // ── 任务调度状态 ────────────────────────────────────────────────────────
    uint32_t active_seq_{0};
    ActionType active_type_{ActionType::IDLE};
    uint64_t active_ticks_{0};
    MechStatus status_{MechStatus::IDLE};

    // ── 归零状态 ────────────────────────────────────────────────────────────
    HomePhase home_phase_{HomePhase::kSeekSwitch};
    double home_hit_angle_{0.0};

    // ── 定位状态 ────────────────────────────────────────────────────────────
    double goto_target_rel_{0.0};

    // ── 软件虚拟零点 ────────────────────────────────────────────────────────
    double calib_zero_offset_{0.0};

    // ── 输入接口 ────────────────────────────────────────────────────────────
    InputInterface<ActionCmd> command_input_;
    InputInterface<double> angle_;
    InputInterface<double> velocity_;
    InputInterface<bool> micro_switch_;

    // ── 输出接口 ────────────────────────────────────────────────────────────
    OutputInterface<double> cmd_velocity_;
    OutputInterface<bool> calibrate_;
    OutputInterface<Feedback> feedback_output_;
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::v2::CarriageActionComponent, rmcs_executor::Component)
