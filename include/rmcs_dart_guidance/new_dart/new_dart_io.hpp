#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ─────────────────────────────────────────────────────────────────────────────
// NewBeltResource
//   新镖架同步带的 IO 包装。
//
//   上位机是决策者：本类直接写两台电机的目标（速度/力矩限幅/标零），
//   并读回编码器与力矩自行判断堵转、到位、超时。
//   下位机 BeltNewController 只做转发 + 左右同步，不含任何判断。
//
//   ⚠️ 本机构是「被弹簧顶住」的竖直滑台，不是靠重力自然下落：
//      电机零电流（NaN）= 弹簧把滑块推回顶部 = 标定零点，
//      这正是故障态想要的停位。所以 hold() 与 release() 语义必须分清，
//      不要凭「重力会掉下来」的直觉把它们混用。
// ─────────────────────────────────────────────────────────────────────────────
class NewBeltResource {
public:
    NewBeltResource(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix = "/dart/belt")
        : status_component_(status_component)
        , command_component_(command_component) {
        command_component_.register_output(
            name_prefix + "/left_motor/cmd_velocity", left_cmd_velocity_, 0.0);
        command_component_.register_output(
            name_prefix + "/right_motor/cmd_velocity", right_cmd_velocity_, 0.0);
        command_component_.register_output(
            name_prefix + "/left_motor/cmd_torque_limit", left_cmd_torque_limit_, 0.0);
        command_component_.register_output(
            name_prefix + "/right_motor/cmd_torque_limit", right_cmd_torque_limit_, 0.0);
        command_component_.register_output(
            name_prefix + "/left_motor/calibrate_request", left_calibrate_, false);
        command_component_.register_output(
            name_prefix + "/right_motor/calibrate_request", right_calibrate_, false);

        status_component_.register_input(name_prefix + "/left_motor/angle", left_angle_, false);
        status_component_.register_input(name_prefix + "/right_motor/angle", right_angle_, false);
        status_component_.register_input(
            name_prefix + "/left_motor/velocity", left_velocity_, false);
        status_component_.register_input(
            name_prefix + "/right_motor/velocity", right_velocity_, false);
        status_component_.register_input(name_prefix + "/left_motor/torque", left_torque_, false);
        status_component_.register_input(
            name_prefix + "/right_motor/torque", right_torque_, false);
    }

    NewBeltResource(const NewBeltResource&) = delete;
    NewBeltResource& operator=(const NewBeltResource&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        const auto bind0 = [&logger](auto& iface, auto value, const char* name) {
            if (iface.ready())
                return;
            iface.make_and_bind_directly(value);
            RCLCPP_WARN(logger, "Failed to fetch \"%s\". Set to default.", name);
        };
        bind0(left_angle_, 0.0, "/dart/belt/left_motor/angle");
        bind0(right_angle_, 0.0, "/dart/belt/right_motor/angle");
        bind0(left_velocity_, 0.0, "/dart/belt/left_motor/velocity");
        bind0(right_velocity_, 0.0, "/dart/belt/right_motor/velocity");
        bind0(left_torque_, 0.0, "/dart/belt/left_motor/torque");
        bind0(right_torque_, 0.0, "/dart/belt/right_motor/torque");
    }

    // ── 写目标 ──────────────────────────────────────────────────────────────
    // 两侧同速：同步带被当作一个执行器推，左右同步由下位机负责。
    void set_velocity(double velocity) {
        *left_cmd_velocity_ = velocity;
        *right_cmd_velocity_ = velocity;
    }

    void set_torque_limit(double limit) {
        *left_cmd_torque_limit_ = limit;
        *right_cmd_torque_limit_ = limit;
    }

    void calibrate(bool on) {
        *left_calibrate_ = on;
        *right_calibrate_ = on;
    }

    // ── 停下：两种语义，别混用 ──────────────────────────────────────────────
    //
    //   hold()     零速 + 力矩限幅 → 速度环把滑块【刹在原地】。
    //              动作正常结束必须用它：降到扳机位后要停在那儿等扳机升起，
    //              放手就会被弹簧顶回顶部，整个供弹序列作废。
    //
    //   release()  速度写 NaN → PID 输出 NaN → 电调零电流 → 电机自由。
    //              零电流 = 弹簧把滑块推回顶部 = 标定零点，
    //              这是故障态（急停 / 任务失败）该有的停位。
    //              注意这不是"掉下来"：机构没有重力自由下落，
    //              刹车反而会把滑块卡在行程中间并离开零点。
    void hold(double torque_limit) {
        set_velocity(0.0);
        set_torque_limit(torque_limit);
        calibrate(false);
    }

    void release() {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        set_velocity(nan);
        set_torque_limit(nan);
        calibrate(false);
    }

    // ── 读反馈 ──────────────────────────────────────────────────────────────
    double left_angle() const { return left_angle_.ready() ? *left_angle_ : 0.0; }
    double right_angle() const { return right_angle_.ready() ? *right_angle_ : 0.0; }
    double left_velocity() const { return left_velocity_.ready() ? *left_velocity_ : 0.0; }
    double right_velocity() const { return right_velocity_.ready() ? *right_velocity_ : 0.0; }
    double left_torque() const { return left_torque_.ready() ? *left_torque_ : 0.0; }
    double right_torque() const { return right_torque_.ready() ? *right_torque_ : 0.0; }

    // 两侧取较大值，避免单侧打滑导致判定偏晚
    double max_abs_velocity() const {
        return std::max(std::abs(left_velocity()), std::abs(right_velocity()));
    }
    double max_abs_torque() const {
        return std::max(std::abs(left_torque()), std::abs(right_torque()));
    }

    // 堵转判定由上位机做：速度掉下去且力矩顶上去
    bool stalled(double velocity_threshold, double torque_threshold) const {
        return max_abs_velocity() < velocity_threshold
            && max_abs_torque() > torque_threshold;
    }

private:
    rmcs_executor::Component& status_component_;
    rmcs_executor::Component& command_component_;

    rmcs_executor::Component::OutputInterface<double> left_cmd_velocity_;
    rmcs_executor::Component::OutputInterface<double> right_cmd_velocity_;
    rmcs_executor::Component::OutputInterface<double> left_cmd_torque_limit_;
    rmcs_executor::Component::OutputInterface<double> right_cmd_torque_limit_;
    rmcs_executor::Component::OutputInterface<bool> left_calibrate_;
    rmcs_executor::Component::OutputInterface<bool> right_calibrate_;

    rmcs_executor::Component::InputInterface<double> left_angle_;
    rmcs_executor::Component::InputInterface<double> right_angle_;
    rmcs_executor::Component::InputInterface<double> left_velocity_;
    rmcs_executor::Component::InputInterface<double> right_velocity_;
    rmcs_executor::Component::InputInterface<double> left_torque_;
    rmcs_executor::Component::InputInterface<double> right_torque_;
};

// ─────────────────────────────────────────────────────────────────────────────
// NewCarriageResource
//   新镖架弹舱的 IO 包装，同 NewBeltResource 的思路：
//   上位机写目标速度 / 标零，读角度与微动开关自行判断归零流程。
// ─────────────────────────────────────────────────────────────────────────────
class NewCarriageResource {
public:
    NewCarriageResource(
        rmcs_executor::Component& status_component, rmcs_executor::Component& command_component,
        const std::string& name_prefix = "/dart/carriage_motor")
        : status_component_(status_component)
        , command_component_(command_component) {
        command_component_.register_output(name_prefix + "/cmd_velocity", cmd_velocity_, 0.0);
        command_component_.register_output(name_prefix + "/calibrate_request", calibrate_, false);

        status_component_.register_input(name_prefix + "/angle", angle_, false);
        status_component_.register_input(name_prefix + "/velocity", velocity_, false);
        status_component_.register_input("/dart/micro_switch", micro_switch_, false);
    }

    NewCarriageResource(const NewCarriageResource&) = delete;
    NewCarriageResource& operator=(const NewCarriageResource&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!angle_.ready()) {
            angle_.make_and_bind_directly(0.0);
            RCLCPP_WARN(logger, "Failed to fetch \"/dart/carriage_motor/angle\". Set to 0.");
        }
        if (!velocity_.ready()) {
            velocity_.make_and_bind_directly(0.0);
            RCLCPP_WARN(logger, "Failed to fetch \"/dart/carriage_motor/velocity\". Set to 0.");
        }
        if (!micro_switch_.ready()) {
            micro_switch_.make_and_bind_directly(false);
            RCLCPP_WARN(logger, "Failed to fetch \"/dart/micro_switch\". Set to false.");
        }
    }

    void set_velocity(double velocity) { *cmd_velocity_ = velocity; }
    void calibrate(bool on) { *calibrate_ = on; }

    // 弹舱是旋转货仓：没有重力，也没有弹簧顶住，位置由电机自己保持。
    // 所以这里保持「零速」，让速度环把货仓刹在当前角度，
    // 不要照抄同步带的 release()（放手只会让货仓在多发的停位之间飘）。
    void hold() {
        set_velocity(0.0);
        calibrate(false);
    }

    double angle() const { return angle_.ready() ? *angle_ : 0.0; }
    double velocity() const { return velocity_.ready() ? *velocity_ : 0.0; }
    bool micro_switch() const { return micro_switch_.ready() && *micro_switch_; }

private:
    rmcs_executor::Component& status_component_;
    rmcs_executor::Component& command_component_;

    rmcs_executor::Component::OutputInterface<double> cmd_velocity_;
    rmcs_executor::Component::OutputInterface<bool> calibrate_;

    rmcs_executor::Component::InputInterface<double> angle_;
    rmcs_executor::Component::InputInterface<double> velocity_;
    rmcs_executor::Component::InputInterface<bool> micro_switch_;
};

} // namespace rmcs_dart_guidance::v2
