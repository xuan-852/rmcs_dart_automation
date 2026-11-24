#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_dart_guidance/msg/new_dart_action.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// BeltActionComponent
//   同步带上层动作执行组件：接收 DartAgent 下发的 (seq, ActionType) 指令，
//   驱动底层规划器/控制器完成放手、保持、上推堵转、相对位移与标零。
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

        // ── 1. 反馈输入 ─────────────────────────────────────────────────────
        register_input("/dart/belt_action/command", command_input_, false);
        register_input("/dart/belt/left_motor/angle", left_angle_, false);
        register_input("/dart/belt/right_motor/angle", right_angle_, false);
        register_input("/dart/belt/stalled", stalled_, false);

        // ── 2. 底层控制输出（经 command_component_ 发出，避免拓扑成环）──────
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
        get_parameter_or("belt_home_velocity", param_home_velocity_, 5.0);
        get_parameter_or("belt_approach_distance", param_approach_distance_, 13.0);
        get_parameter_or("belt_approach_velocity", param_approach_velocity_, 3.0);
        get_parameter_or("belt_down_velocity", param_down_velocity_, 10.0);
        get_parameter_or("belt_up_velocity", param_up_velocity_, 8.0);
        get_parameter_or("homing_torque_limit", param_homing_torque_limit_, 3.0);
        get_parameter_or("belt_load_torque_limit", param_load_torque_limit_, 6.0);
        get_parameter_or("belt_unload_torque_limit", param_unload_torque_limit_, 2.0);
        get_parameter_or("arrive_tolerance", param_arrive_tolerance_, 0.03);
    }

    BeltActionComponent(const BeltActionComponent&) = delete;
    BeltActionComponent& operator=(const BeltActionComponent&) = delete;
    BeltActionComponent(BeltActionComponent&&) = delete;
    BeltActionComponent& operator=(BeltActionComponent&&) = delete;

    ~BeltActionComponent() override = default;

    void before_updating() override {
        bind_or(command_input_, ActionCmd{}, "/dart/belt_action/command");
        bind_or(left_angle_, 0.0, "/dart/belt/left_motor/angle");
        bind_or(right_angle_, 0.0, "/dart/belt/right_motor/angle");
        bind_or(stalled_, false, "/dart/belt/stalled");
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
    class CommandComponent : public rmcs_executor::Component {
    public:
        void update() override {}
    };

    template <typename T, typename V>
    void bind_or(InputInterface<T>& iface, V&& fallback, const char* name) {
        if (iface.ready())
            return;
        iface.make_and_bind_directly(T(std::forward<V>(fallback)));
        RCLCPP_WARN(get_logger(), "Failed to fetch \"%s\". Set to default.", name);
    }

    static double pick_positive(double override_val, double fallback) {
        return (std::isfinite(override_val) && override_val > 0.0) ? override_val : fallback;
    }

    static double pick_nonzero(double override_val, double fallback) {
        return (std::isfinite(override_val) && override_val != 0.0) ? override_val : fallback;
    }

    MechStatus dispatch_action(const ActionCmd& cmd, bool is_new) {
        switch (active_type_) {
        case ActionType::IDLE:
            return MechStatus::IDLE;

        case ActionType::RELEASE:
            drive_release();
            return MechStatus::SUCCEEDED;

        case ActionType::HOLD:
            drive_hold(pick_positive(cmd.torque_limit, param_unload_torque_limit_));
            return MechStatus::SUCCEEDED;

        case ActionType::HOME_TO_STALL:
            return run_push_to_stall(
                cmd, is_new, param_home_velocity_, param_homing_torque_limit_);

        case ActionType::RETURN_TO_STALL:
            return run_push_to_stall(
                cmd, is_new, param_up_velocity_, param_unload_torque_limit_);

        case ActionType::MOVE_REL:
            return run_move_rel(cmd, is_new);

        case ActionType::PULL_TO_TRIGGER:
            return run_pull_to_trigger(cmd, is_new);

        case ActionType::CALIBRATE:
            return run_calibrate(cmd, is_new);
        }
        return MechStatus::IDLE;
    }

    // ── 动作：上推至顶部机械限位堵转（HOME_TO_STALL / RETURN_TO_STALL）────────
    MechStatus run_push_to_stall(
        const ActionCmd& cmd, bool is_new, double default_vel, double default_torque_limit) {
        if (is_new) {
            stall_velocity_ = pick_nonzero(cmd.velocity, default_vel);
            stall_torque_limit_ = pick_positive(cmd.torque_limit, default_torque_limit);
        }

        drive_velocity(stall_velocity_, stall_torque_limit_);
        // 跳过起步前 kStallGraceTicks 毫秒的加速瞬间，确认堵转后原地顶住
        if (active_ticks_ >= kStallGraceTicks && *stalled_) {
            drive_hold(stall_torque_limit_);
            return MechStatus::SUCCEEDED;
        }
        return MechStatus::BUSY;
    }

    bool init_move(const ActionCmd& cmd, double default_vel) {
        if (!std::isfinite(cmd.distance) || std::abs(cmd.distance) <= 1e-6) {
            drive_hold(param_load_torque_limit_);
            RCLCPP_ERROR(get_logger(), "[BeltAction] invalid move distance=%.4f", cmd.distance);
            return false;
        }
        move_signed_distance_ = cmd.distance;
        move_velocity_limit_ = pick_positive(cmd.velocity, default_vel);
        move_torque_limit_ = pick_positive(cmd.torque_limit, param_load_torque_limit_);
        move_start_left_ = *left_angle_;
        move_start_right_ = *right_angle_;
        move_target_left_ = move_start_left_ + move_signed_distance_;
        move_target_right_ = move_start_right_ + move_signed_distance_;
        return true;
    }

    std::pair<double, double> moved_distances() const {
        const double dir = move_signed_distance_ >= 0.0 ? 1.0 : -1.0;
        return {(*left_angle_ - move_start_left_) * dir, (*right_angle_ - move_start_right_) * dir};
    }

    // ── 动作：相对当前位置移动指定弧度（走 TrapezoidPlanner）──────────────────
    MechStatus run_move_rel(const ActionCmd& cmd, bool is_new) {
        if (is_new) {
            const double default_vel =
                cmd.distance >= 0.0 ? param_up_velocity_ : param_down_velocity_;
            if (!init_move(cmd, default_vel))
                return MechStatus::FAILED;
        }

        drive_position(
            move_target_left_, move_target_right_, move_velocity_limit_, move_torque_limit_);

        const auto [moved_l, moved_r] = moved_distances();
        if (std::max(moved_l, moved_r) >= std::abs(move_signed_distance_) - param_arrive_tolerance_) {
            drive_hold(move_torque_limit_);
            return MechStatus::SUCCEEDED;
        }
        return MechStatus::BUSY;
    }

    // ── 动作：分段下拉（低速接近碰撞点 → 不断速加速拉弹簧 → 到底前减速）─────
    MechStatus run_pull_to_trigger(const ActionCmd& cmd, bool is_new) {
        if (is_new) {
            if (!init_move(cmd, param_down_velocity_))
                return MechStatus::FAILED;
            pull_approach_distance_ =
                pick_positive(cmd.approach_distance, param_approach_distance_);
            pull_approach_velocity_ =
                pick_positive(cmd.approach_velocity, param_approach_velocity_);
        }

        const auto [moved_l, moved_r] = moved_distances();
        const bool past_collision = std::min(moved_l, moved_r) >= pull_approach_distance_;
        const double active_vel = past_collision ? move_velocity_limit_ : pull_approach_velocity_;

        drive_position(move_target_left_, move_target_right_, active_vel, move_torque_limit_);

        if (std::max(moved_l, moved_r) >= std::abs(move_signed_distance_) - param_arrive_tolerance_) {
            drive_hold(move_torque_limit_);
            return MechStatus::SUCCEEDED;
        }
        return MechStatus::BUSY;
    }

    // ── 动作：标零（记录软件虚拟零点 + 通知下层同步控制器对齐零点）─────────
    MechStatus run_calibrate(const ActionCmd& cmd, bool is_new) {
        if (is_new) {
            calib_torque_limit_ = pick_positive(cmd.torque_limit, param_homing_torque_limit_);
            calib_left_zero_offset_ = *left_angle_;
            calib_right_zero_offset_ = *right_angle_;
        }

        const bool finished = active_ticks_ >= kCalibrateHoldTicks;
        drive_velocity(0.0, calib_torque_limit_, /*calibrate=*/!finished);
        return finished ? MechStatus::SUCCEEDED : MechStatus::BUSY;
    }

    // ── 底层输出收口 ────────────────────────────────────────────────────────
    void apply_outputs(
        double target_l, double target_r, double direct_vel, double vel_limit, double torque_limit,
        bool calibrate) {
        *left_target_angle_ = target_l;
        *right_target_angle_ = target_r;
        *left_direct_velocity_ = direct_vel;
        *right_direct_velocity_ = direct_vel;
        *left_velocity_limit_ = vel_limit;
        *right_velocity_limit_ = vel_limit;
        *left_torque_limit_ = torque_limit;
        *right_torque_limit_ = torque_limit;
        *left_calibrate_ = calibrate;
        *right_calibrate_ = calibrate;
    }

    void drive_release() {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double inf = std::numeric_limits<double>::infinity();
        apply_outputs(nan, nan, nan, inf, nan, false);
    }

    void drive_velocity(double velocity, double torque_limit, bool calibrate = false) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        constexpr double inf = std::numeric_limits<double>::infinity();
        apply_outputs(nan, nan, velocity, inf, torque_limit, calibrate);
    }

    void drive_hold(double torque_limit) { drive_velocity(0.0, torque_limit); }

    void drive_position(
        double target_l, double target_r, double velocity_limit, double torque_limit) {
        constexpr double nan = std::numeric_limits<double>::quiet_NaN();
        apply_outputs(target_l, target_r, nan, std::abs(velocity_limit), torque_limit, false);
    }

    static constexpr uint64_t kStallGraceTicks = 50;
    static constexpr uint64_t kCalibrateHoldTicks = 5;

    // ── 参数 ────────────────────────────────────────────────────────────────
    double param_home_velocity_{5.0};
    double param_approach_distance_{13.0};
    double param_approach_velocity_{3.0};
    double param_down_velocity_{10.0};
    double param_up_velocity_{8.0};
    double param_homing_torque_limit_{3.0};
    double param_load_torque_limit_{6.0};
    double param_unload_torque_limit_{2.0};
    double param_arrive_tolerance_{0.03};

    // ── 任务调度状态 ────────────────────────────────────────────────────────
    std::shared_ptr<CommandComponent> command_component_;
    uint32_t active_seq_{0};
    ActionType active_type_{ActionType::IDLE};
    uint64_t active_ticks_{0};
    MechStatus status_{MechStatus::IDLE};

    // ── 堵转推进状态 ────────────────────────────────────────────────────────
    double stall_velocity_{0.0};
    double stall_torque_limit_{0.0};

    // ── 相对位移 / 分段下拉状态 ─────────────────────────────────────────────
    double move_signed_distance_{0.0};
    double move_velocity_limit_{0.0};
    double move_torque_limit_{0.0};
    double move_start_left_{0.0};
    double move_start_right_{0.0};
    double move_target_left_{0.0};
    double move_target_right_{0.0};
    double pull_approach_distance_{13.0};
    double pull_approach_velocity_{3.0};

    // ── 软件虚拟零点 ────────────────────────────────────────────────────────
    double calib_torque_limit_{0.0};
    double calib_left_zero_offset_{0.0};
    double calib_right_zero_offset_{0.0};

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
