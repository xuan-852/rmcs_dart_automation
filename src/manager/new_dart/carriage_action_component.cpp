#include <algorithm>
#include <cmath>
#include <cstdint>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/new_dart_action.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// CarriageActionComponent
//   弹舱上层动作执行组件（标准 RMCS 写法）。
//
//   职责：
//     接收 DartAgent 发来的通用能力指令（HOLD / HOME / GOTO / CALIBRATE），
//     按 seq 订单号完成撞开关退离、角度定位与标零脉冲，底层 CarriageNewController
//     仍保持纯执行器（手动覆盖 + 转发）。
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
        // ── 1. 注册输入 ─────────────────────────────────────────────────────
        register_input("/dart/carriage_action/command", command_input_, false);
        register_input("/dart/carriage_motor/angle", angle_, false);
        register_input("/dart/carriage_motor/velocity", velocity_, false);
        register_input("/dart/micro_switch", micro_switch_, false);

        // ── 2. 注册输出 ─────────────────────────────────────────────────────
        register_output("/dart/carriage_motor/cmd_velocity", cmd_velocity_, 0.0);
        register_output("/dart/carriage_motor/calibrate_request", calibrate_, false);
        register_output("/dart/carriage_action/feedback", feedback_output_, Feedback{});

        // ── 3. 读取参数 ─────────────────────────────────────────────────────
        get_parameter_or("carriage_home_velocity", home_velocity_, 5.0);
        get_parameter_or("carriage_back_off_velocity", back_off_velocity_, 2.0);
        get_parameter_or("carriage_back_off_distance", back_off_distance_, 0.1);
        get_parameter_or("carriage_goto_velocity", goto_velocity_, 20.0);
        get_parameter_or("carriage_goto_gain", goto_gain_, 10.0);
        get_parameter_or("carriage_goto_tolerance", goto_tolerance_, 0.02);
    }

    CarriageActionComponent(const CarriageActionComponent&) = delete;
    CarriageActionComponent& operator=(const CarriageActionComponent&) = delete;
    CarriageActionComponent(CarriageActionComponent&&) = delete;
    CarriageActionComponent& operator=(CarriageActionComponent&&) = delete;

    ~CarriageActionComponent() override = default;

    void before_updating() override {
        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(ActionCmd{});
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/carriage_action/command\". Set to IDLE.");
        }
        if (!angle_.ready()) {
            angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/carriage_motor/angle\". Set to 0.");
        }
        if (!velocity_.ready()) {
            velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/carriage_motor/velocity\". Set to 0.");
        }
        if (!micro_switch_.ready()) {
            micro_switch_.make_and_bind_directly(false);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/micro_switch\". Set to false.");
        }
    }

    void update() override {
        const ActionCmd cmd = *command_input_;
        if (cmd.seq != active_seq_)
            start_command(cmd);

        if (status_ == MechStatus::BUSY)
            step_command();

        *feedback_output_ = Feedback{active_seq_, status_};
    }

private:
    void start_command(const ActionCmd& cmd) {
        active_seq_ = cmd.seq;
        active_type_ = cmd.type;
        ticks_ = 0;

        switch (cmd.type) {
        case ActionType::IDLE:
            status_ = MechStatus::IDLE;
            break;

        case ActionType::HOLD:
            hold();
            status_ = MechStatus::SUCCEEDED;
            break;

        case ActionType::HOME:
            home_phase_ = 0;
            hit_angle_ = 0.0;
            *cmd_velocity_ = home_velocity_;
            *calibrate_ = false;
            status_ = MechStatus::BUSY;
            break;

        case ActionType::GOTO:
            if (!std::isfinite(cmd.target_angle)) {
                hold();
                status_ = MechStatus::FAILED;
                RCLCPP_ERROR(get_logger(), "[CarriageAction] invalid GOTO target_angle");
                return;
            }
            target_angle_ = cmd.target_angle;
            *calibrate_ = false;
            status_ = MechStatus::BUSY;
            break;

        case ActionType::CALIBRATE:
            *cmd_velocity_ = 0.0;
            *calibrate_ = true;
            status_ = MechStatus::BUSY;
            break;
        }
    }

    void step_command() {
        ++ticks_;

        switch (active_type_) {
        case ActionType::HOME:
            step_home();
            break;

        case ActionType::GOTO: {
            const double err = target_angle_ - *angle_;
            if (std::abs(err) <= std::abs(goto_tolerance_)) {
                hold();
                status_ = MechStatus::SUCCEEDED;
                break;
            }
            const double max_v = std::abs(goto_velocity_);
            *cmd_velocity_ = std::clamp(err * goto_gain_, -max_v, max_v);
            break;
        }

        case ActionType::CALIBRATE:
            if (ticks_ >= kCalibrateHoldTicks) {
                hold();
                status_ = MechStatus::SUCCEEDED;
            }
            break;

        default:
            break;
        }
    }

    void step_home() {
        if (home_phase_ == 0) {
            *cmd_velocity_ = home_velocity_;
            if (*micro_switch_) {
                hit_angle_ = *angle_;
                home_phase_ = 1;
                RCLCPP_INFO(
                    get_logger(), "[CarriageAction] switch hit at %.3f, backing off", hit_angle_);
            } else if (ticks_ % 1000 == 0) {
                RCLCPP_INFO(
                    get_logger(), "[CarriageAction] seeking: angle=%.3f vel=%.3f", *angle_,
                    *velocity_);
            }
            return;
        }

        *cmd_velocity_ = back_off_velocity_;
        const double moved = std::abs(*angle_ - hit_angle_);
        if (moved >= std::abs(back_off_distance_)) {
            hold();
            status_ = MechStatus::SUCCEEDED;
            RCLCPP_INFO(get_logger(), "[CarriageAction] backed off %.3f, done", moved);
            return;
        }
        if (ticks_ % 1000 == 0) {
            RCLCPP_INFO(
                get_logger(), "[CarriageAction] backing off: %.4f / %.4f", moved,
                std::abs(back_off_distance_));
        }
    }

    void hold() {
        *cmd_velocity_ = 0.0;
        *calibrate_ = false;
    }

    static constexpr uint64_t kCalibrateHoldTicks = 5;

    // ── 参数 ────────────────────────────────────────────────────────────────
    double home_velocity_{5.0};
    double back_off_velocity_{2.0};
    double back_off_distance_{0.1};
    double goto_velocity_{20.0};
    double goto_gain_{10.0};
    double goto_tolerance_{0.02};

    // ── 状态变量 ────────────────────────────────────────────────────────────
    uint32_t active_seq_{0};
    ActionType active_type_{ActionType::IDLE};
    MechStatus status_{MechStatus::IDLE};
    uint64_t ticks_{0};
    int home_phase_{0};
    double hit_angle_{0.0};
    double target_angle_{0.0};

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
