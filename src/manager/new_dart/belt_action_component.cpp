#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/new_dart_action.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// BeltActionComponent
//   同步带上层动作执行组件（标准 RMCS 写法）。
//
//   职责：
//     接收 DartAgent 发来的通用物理能力指令（RELEASE / HOLD / HOME_TO_STALL /
//     RETURN_TO_STALL / MOVE_REL / CALIBRATE），按 seq 订单号执行动作并回报反馈。
//     底层手动控制、左右同步、物理堵转检测仍全部留在 BeltNewController，互不耦合。
// ═════════════════════════════════════════════════════════════════════════════
class BeltActionComponent
    : public rmcs_executor::Component
    , public rclcpp::Node {
    using ActionCmd = rmcs_dart_guidance::msg::BeltActionCommand;
    using ActionType = rmcs_dart_guidance::msg::BeltActionType;
    using Feedback = rmcs_dart_guidance::msg::ActionFeedback;
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;

public:
    BeltActionComponent()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , command_component_(
              create_partner_component<CommandComponent>(get_component_name() + "_command")) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double inf = std::numeric_limits<double>::infinity();

        // ── 1. 注册输入 ─────────────────────────────────────────────────────
        register_input("/dart/belt_action/command", command_input_, false);
        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/stalled", stalled_, false);

        // ── 2. 注册输出（底层控制输出经 command_component_ 发出，避免拓扑成环）
        command_component_->register_output("/dart/belt/left_motor/target_angle", left_target_angle_, nan);
        command_component_->register_output("/dart/belt/right_motor/target_angle", right_target_angle_, nan);
        command_component_->register_output("/dart/belt/left_motor/direct_velocity", left_direct_velocity_, nan);
        command_component_->register_output("/dart/belt/right_motor/direct_velocity", right_direct_velocity_, nan);
        command_component_->register_output("/dart/belt/left_motor/velocity_limit", left_velocity_limit_, inf);
        command_component_->register_output("/dart/belt/right_motor/velocity_limit", right_velocity_limit_, inf);
        command_component_->register_output("/dart/belt/left_motor/cmd_torque_limit", left_torque_limit_, 0.0);
        command_component_->register_output("/dart/belt/right_motor/cmd_torque_limit", right_torque_limit_, 0.0);
        command_component_->register_output("/dart/belt/left_motor/calibrate_request", left_calibrate_, false);
        command_component_->register_output("/dart/belt/right_motor/calibrate_request", right_calibrate_, false);

        register_output("/dart/belt_action/feedback", feedback_output_, Feedback{});

        // ── 3. 读取参数 ─────────────────────────────────────────────────────
        get_parameter_or("belt_home_velocity", belt_home_velocity_, 5.0);
        get_parameter_or("belt_down_velocity", belt_down_velocity_, 5.0);
        get_parameter_or("belt_up_velocity", belt_up_velocity_, 8.0);
        get_parameter_or("homing_torque_limit", homing_torque_limit_, 3.0);
        get_parameter_or("belt_load_torque_limit", belt_load_torque_limit_, 6.0);
        get_parameter_or("belt_unload_torque_limit", belt_unload_torque_limit_, 2.0);
        get_parameter_or("arrive_tolerance", arrive_tolerance_, 0.03);
    }

    BeltActionComponent(const BeltActionComponent&) = delete;
    BeltActionComponent& operator=(const BeltActionComponent&) = delete;
    BeltActionComponent(BeltActionComponent&&) = delete;
    BeltActionComponent& operator=(BeltActionComponent&&) = delete;

    ~BeltActionComponent() override = default;

    void before_updating() override {
        if (!command_input_.ready()) {
            command_input_.make_and_bind_directly(ActionCmd{});
            RCLCPP_WARN(
                get_logger(), "Failed to fetch \"/dart/belt_action/command\". Set to IDLE.");
        }
        if (!left_angle_.ready()) {
            left_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/belt/left_motor/angle\". Set to 0.");
        }
        if (!right_angle_.ready()) {
            right_angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/belt/right_motor/angle\". Set to 0.");
        }
        if (!stalled_.ready()) {
            stalled_.make_and_bind_directly(false);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/dart/belt/stalled\". Set to false.");
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
    class CommandComponent : public rmcs_executor::Component {
    public:
        void update() override {}
    };

    static double pick(double override_val, double fallback) {
        return (std::isfinite(override_val) && override_val > 0.0) ? override_val : fallback;
    }

    void start_command(const ActionCmd& cmd) {
        active_seq_ = cmd.seq;
        active_type_ = cmd.type;
        ticks_ = 0;

        switch (cmd.type) {
        case ActionType::IDLE:
            status_ = MechStatus::IDLE;
            break;

        case ActionType::RELEASE:
            release_belt();
            status_ = MechStatus::SUCCEEDED;
            break;

        case ActionType::HOLD:
            active_torque_limit_ = pick(cmd.torque_limit, belt_unload_torque_limit_);
            hold_belt(active_torque_limit_);
            status_ = MechStatus::SUCCEEDED;
            break;

        case ActionType::HOME_TO_STALL:
            active_velocity_ = cmd.velocity != 0.0 && std::isfinite(cmd.velocity)
                                 ? cmd.velocity
                                 : belt_home_velocity_;
            active_torque_limit_ = pick(cmd.torque_limit, homing_torque_limit_);
            write_direct(active_velocity_, active_torque_limit_);
            status_ = MechStatus::BUSY;
            break;

        case ActionType::RETURN_TO_STALL:
            active_velocity_ = cmd.velocity != 0.0 && std::isfinite(cmd.velocity)
                                 ? cmd.velocity
                                 : belt_up_velocity_;
            active_torque_limit_ = pick(cmd.torque_limit, belt_unload_torque_limit_);
            write_direct(active_velocity_, active_torque_limit_);
            status_ = MechStatus::BUSY;
            break;

        case ActionType::MOVE_REL:
            if (!std::isfinite(cmd.distance) || std::abs(cmd.distance) <= 1e-6) {
                hold_belt(belt_load_torque_limit_);
                status_ = MechStatus::FAILED;
                RCLCPP_ERROR(get_logger(), "[BeltAction] invalid MOVE_REL distance=%.4f", cmd.distance);
                return;
            }
            signed_distance_ = cmd.distance;
            active_velocity_ = pick(
                cmd.velocity, signed_distance_ >= 0.0 ? belt_up_velocity_ : belt_down_velocity_);
            active_torque_limit_ = pick(cmd.torque_limit, belt_load_torque_limit_);
            start_left_ = *left_angle_;
            start_right_ = *right_angle_;
            target_left_ = start_left_ + signed_distance_;
            target_right_ = start_right_ + signed_distance_;
            write_move_to(target_left_, target_right_, active_velocity_, active_torque_limit_);
            status_ = MechStatus::BUSY;
            break;

        case ActionType::CALIBRATE:
            active_torque_limit_ = pick(cmd.torque_limit, homing_torque_limit_);
            hold_belt(active_torque_limit_);
            *left_calibrate_ = true;
            *right_calibrate_ = true;
            status_ = MechStatus::BUSY;
            break;
        }
    }

    void step_command() {
        ++ticks_;

        switch (active_type_) {
        case ActionType::HOME_TO_STALL:
        case ActionType::RETURN_TO_STALL:
            write_direct(active_velocity_, active_torque_limit_);
            if (ticks_ >= kStallGraceTicks && *stalled_) {
                hold_belt(active_torque_limit_);
                status_ = MechStatus::SUCCEEDED;
            }
            break;

        case ActionType::MOVE_REL: {
            write_move_to(target_left_, target_right_, active_velocity_, active_torque_limit_);
            const double dir = signed_distance_ >= 0.0 ? 1.0 : -1.0;
            const double moved_l = (*left_angle_ - start_left_) * dir;
            const double moved_r = (*right_angle_ - start_right_) * dir;
            if (std::max(moved_l, moved_r) >= std::abs(signed_distance_) - arrive_tolerance_) {
                hold_belt(active_torque_limit_);
                status_ = MechStatus::SUCCEEDED;
            }
            break;
        }

        case ActionType::CALIBRATE:
            if (ticks_ >= kCalibrateHoldTicks) {
                hold_belt(active_torque_limit_);
                status_ = MechStatus::SUCCEEDED;
            }
            break;

        default:
            break;
        }
    }

    void write_move_to(
        double target_l, double target_r, double velocity_limit, double torque_limit) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        *left_target_angle_ = target_l;
        *right_target_angle_ = target_r;
        *left_direct_velocity_ = nan;
        *right_direct_velocity_ = nan;
        *left_velocity_limit_ = std::abs(velocity_limit);
        *right_velocity_limit_ = std::abs(velocity_limit);
        *left_torque_limit_ = torque_limit;
        *right_torque_limit_ = torque_limit;
        *left_calibrate_ = false;
        *right_calibrate_ = false;
    }

    void write_direct(double velocity, double torque_limit) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double inf = std::numeric_limits<double>::infinity();
        *left_target_angle_ = nan;
        *right_target_angle_ = nan;
        *left_direct_velocity_ = velocity;
        *right_direct_velocity_ = velocity;
        *left_velocity_limit_ = inf;
        *right_velocity_limit_ = inf;
        *left_torque_limit_ = torque_limit;
        *right_torque_limit_ = torque_limit;
        *left_calibrate_ = false;
        *right_calibrate_ = false;
    }

    void hold_belt(double torque_limit) { write_direct(0.0, torque_limit); }

    void release_belt() {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double inf = std::numeric_limits<double>::infinity();
        *left_target_angle_ = nan;
        *right_target_angle_ = nan;
        *left_direct_velocity_ = nan;
        *right_direct_velocity_ = nan;
        *left_velocity_limit_ = inf;
        *right_velocity_limit_ = inf;
        *left_torque_limit_ = nan;
        *right_torque_limit_ = nan;
        *left_calibrate_ = false;
        *right_calibrate_ = false;
    }

    static constexpr uint64_t kStallGraceTicks = 50;
    static constexpr uint64_t kCalibrateHoldTicks = 5;

    // ── 参数 ────────────────────────────────────────────────────────────────
    double belt_home_velocity_{5.0};
    double belt_down_velocity_{5.0};
    double belt_up_velocity_{8.0};
    double homing_torque_limit_{3.0};
    double belt_load_torque_limit_{6.0};
    double belt_unload_torque_limit_{2.0};
    double arrive_tolerance_{0.03};

    // ── 状态变量 ────────────────────────────────────────────────────────────
    std::shared_ptr<CommandComponent> command_component_;
    uint32_t active_seq_{0};
    ActionType active_type_{ActionType::IDLE};
    MechStatus status_{MechStatus::IDLE};
    uint64_t ticks_{0};

    double signed_distance_{0.0};
    double active_velocity_{0.0};
    double active_torque_limit_{0.0};
    double start_left_{0.0};
    double start_right_{0.0};
    double target_left_{0.0};
    double target_right_{0.0};

    // ── 输入接口 ────────────────────────────────────────────────────────────
    InputInterface<ActionCmd> command_input_;
    InputInterface<double> left_angle_;
    InputInterface<double> right_angle_;
    InputInterface<bool> stalled_;

    // ── 输出接口 ────────────────────────────────────────────────────────────
    OutputInterface<double> left_target_angle_;
    OutputInterface<double> right_target_angle_;
    OutputInterface<double> left_direct_velocity_;
    OutputInterface<double> right_direct_velocity_;
    OutputInterface<double> left_velocity_limit_;
    OutputInterface<double> right_velocity_limit_;
    OutputInterface<double> left_torque_limit_;
    OutputInterface<double> right_torque_limit_;
    OutputInterface<bool> left_calibrate_;
    OutputInterface<bool> right_calibrate_;
    OutputInterface<Feedback> feedback_output_;
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::v2::BeltActionComponent, rmcs_executor::Component)
