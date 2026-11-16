#pragma once

#include "manager/runtime/action.hpp"

#include <rmcs_dart_guidance/new_dart/new_dart_io.hpp>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// 同步带动作
//
// 下位机不含任何判断，所以「走多远」「到没到」全部由这里的动作自己算：
//   on_enter   记录起点角度，写目标速度
//   update     读角度算已走距离，够数就成功，超时报失败
//   on_exit    零速保持（刹住，不下电）—— 成功时机构必须停在当前位置
//   on_failure 放手（NaN）—— 故障时让弹簧把滑块顶回零点，理由见 NewBeltResource::release()
// ═════════════════════════════════════════════════════════════════════════════

// ── 相对位移：从起点走够 distance ────────────────────────────────────────────
//
// 这里刻意不判堵转。原因：
//   推到扳机位时推块与扳机解耦（舵机已抬起），但推块仍可能被飞镖残骸或
//   机械摩擦顶住。中途"堵转"既可能是真卡死，也可能是正常顶到位。
//   判定责任交给超时：走不够 distance 才会超时报错，走够了就成功。
//   真要早期发现卡死，应由上层另加监控，不在这里猜。
class BeltMoveAction : public manager::IAction {
public:
    BeltMoveAction(
        std::string name, NewBeltResource& belt, double velocity, double distance,
        double torque_limit, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , belt_(belt)
        , velocity_(velocity)
        , distance_(std::abs(distance))
        , torque_limit_(torque_limit)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        start_left_ = belt_.left_angle();
        start_right_ = belt_.right_angle();
        belt_.set_torque_limit(torque_limit_);
        belt_.set_velocity(velocity_);
    }

    manager::ActionStatus update() override {
        // 每 tick 重写目标（见 CarriageHomeAction 的说明）
        belt_.set_torque_limit(torque_limit_);
        belt_.set_velocity(velocity_);

        if (traveled() >= distance_)
            return manager::ActionStatus::SUCCESS;
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_)
            return fail(manager::ActionFailureReason::TIMEOUT);
        return manager::ActionStatus::RUNNING;
    }

    void on_exit() override { belt_.hold(torque_limit_); }

    // 失败时放手：弹簧把滑块顶回零点，停下位置确定可预期。
    // 若沿用默认的 on_failure()（= on_exit 刹车），滑块会被卡在半路，
    // 下次 dart-init 的「上推堵转找顶」起点就不可知了。
    void on_failure() override { belt_.release(); }

private:
    // 两侧取 max：只要有一侧走够就算到位，避免打滑侧拖累整体卡在超时
    double traveled() const {
        const double l = std::abs(belt_.left_angle() - start_left_);
        const double r = std::abs(belt_.right_angle() - start_right_);
        return std::max(l, r);
    }

    NewBeltResource& belt_;
    double velocity_;
    double distance_;
    double torque_limit_;
    uint64_t timeout_ticks_;
    double start_left_{0.0};
    double start_right_{0.0};
};

// ── 转到堵转：一直往上/下推，堵转即成功（回顶、找限位用）──────────────────────
class BeltStallAction : public manager::IAction {
public:
    BeltStallAction(
        std::string name, NewBeltResource& belt, double velocity, double torque_limit,
        uint64_t timeout_ticks)
        : IAction(std::move(name))
        , belt_(belt)
        , velocity_(velocity)
        , torque_limit_(torque_limit)
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        stall_ticks_ = 0;
        belt_.set_torque_limit(torque_limit_);
        belt_.set_velocity(velocity_);
    }

    manager::ActionStatus update() override {
        // 每 tick 重写目标，理由同 CarriageHomeAction::update()：
        // 输出接口理论上保持值，但不能假设"没有人把它清掉"。
        // 之前只在 on_enter 写一次，实测 carriage 因此完全不动。
        belt_.set_torque_limit(torque_limit_);
        belt_.set_velocity(velocity_);

        if (belt_.stalled(kStallVelocity, stall_torque_threshold())) {
            if (++stall_ticks_ >= kStallConfirmTicks)
                return manager::ActionStatus::SUCCESS;
        } else {
            stall_ticks_ = 0;
        }
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_)
            return fail(manager::ActionFailureReason::TIMEOUT);
        return manager::ActionStatus::RUNNING;
    }

    void on_exit() override { belt_.hold(torque_limit_); }

    // 失败（含超时）时放手，同 BeltMoveAction：让弹簧把滑块顶回零点
    void on_failure() override { belt_.release(); }

private:
    // 堵转阈值随力矩限幅走：顶住了力矩一定会爬到限幅，
    // 用固定值会在限幅调小时永远判不出来。
    double stall_torque_threshold() const { return std::abs(torque_limit_) * kStallTorqueRatio; }

    static constexpr double kStallVelocity = 0.05;
    static constexpr double kStallTorqueRatio = 0.8;  // 达到限幅 80% 即视为顶住
    static constexpr int kStallConfirmTicks = 50;     // 连续 50ms 判堵转

    NewBeltResource& belt_;
    double velocity_;
    double torque_limit_;
    uint64_t timeout_ticks_;
    int stall_ticks_{0};
};

// ── 标零：把当前位置设为 0 点 ───────────────────────────────────────────────
// 硬件层按上升沿生效，这里保持 calibrate 打一拍再放开，确保被采样到。
class BeltCalibrateAction : public manager::IAction {
public:
    explicit BeltCalibrateAction(std::string name, NewBeltResource& belt, double torque_limit)
        : IAction(std::move(name))
        , belt_(belt)
        , torque_limit_(torque_limit) {}

    void on_enter() override {
        belt_.hold(torque_limit_);
        belt_.calibrate(true);
    }

    manager::ActionStatus update() override {
        if (elapsed_ticks() >= kCalibrateTicks)
            return manager::ActionStatus::SUCCESS;
        return manager::ActionStatus::RUNNING;
    }

    void on_exit() override {
        belt_.calibrate(false);
        belt_.hold(torque_limit_);
    }

    // 标零本身不会动，失败通常意味着标零请求没被硬件层采到。
    // 此时不要留在原地刹车：放手回顶，至少让下一次归零从确定位置开始。
    void on_failure() override {
        belt_.calibrate(false);
        belt_.release();
    }

private:
    static constexpr uint64_t kCalibrateTicks = 5;

    NewBeltResource& belt_;
    double torque_limit_;
};

// ═════════════════════════════════════════════════════════════════════════════
// 弹舱动作
// ═════════════════════════════════════════════════════════════════════════════

// ── 归零找开关：向上撞微动开关 → 反向退离 ────────────────────────────────────
// 老镖架的 CARRIAGE_CALIBRATE 就是这个过程，现在搬到了上位机：
//   kFindSwitch  向上匀速撞开关
//   kBackOff     开关触发后反向退离一段
//
// 只做「找到机械零位」这件事，不标零。
// 标零由任务里统一的 carriage_calibrate() 完成，和 belt 对称 ——
// 谁什么时候标零，在 task 里一眼看得见，不藏在动作内部。
class CarriageHomeAction : public manager::IAction {
public:
    CarriageHomeAction(
        std::string name, NewCarriageResource& carriage, double up_velocity,
        double back_off_velocity, double back_off_distance, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , carriage_(carriage)
        , up_velocity_(up_velocity)
        , back_off_velocity_(back_off_velocity)
        , back_off_distance_(std::abs(back_off_distance))
        , timeout_ticks_(timeout_ticks) {}

    void on_enter() override {
        phase_ = 0;
        hit_angle_ = 0.0;
        carriage_.set_velocity(up_velocity_);
    }

    manager::ActionStatus update() override {
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_) {
            carriage_.hold();
            return fail(manager::ActionFailureReason::TIMEOUT);
        }

        // 每 tick 重写速度目标。
        // 之前只在 on_enter 写一次，实测弹舱完全不动：on_enter 由 tick_first()
        // 调用一次，之后 tick() 只调 update()，目标一旦被清就再也写不回去。
        // 重写成本极低，换来"目标一定在生效"的强保证。
        switch (phase_) {
        case 0:  // 找开关
            carriage_.set_velocity(up_velocity_);
            if (carriage_.micro_switch()) {
                hit_angle_ = carriage_.angle();
                phase_ = 1;
                log("switch hit at angle=%.3f, back off to %.3f rad/s", hit_angle_, back_off_velocity_);
            } else if (elapsed_ticks() % 1000 == 0) {
                // 低速时进度慢，每秒报一次，现场一眼看出是"没转"还是"转得慢"
                log("seeking: angle=%.3f vel=%.3f", carriage_.angle(), carriage_.velocity());
            }
            return manager::ActionStatus::RUNNING;

        default:  // 退离
            carriage_.set_velocity(back_off_velocity_);
            if (std::abs(carriage_.angle() - hit_angle_) >= back_off_distance_) {
                carriage_.hold();
                log("backed off by %.3f, done", std::abs(carriage_.angle() - hit_angle_));
                return manager::ActionStatus::SUCCESS;
            }
            if (elapsed_ticks() % 1000 == 0) {
                log(
                    "backing off: moved=%.4f/%.4f angle=%.3f",
                    std::abs(carriage_.angle() - hit_angle_), back_off_distance_,
                    carriage_.angle());
            }
            return manager::ActionStatus::RUNNING;
        }
    }

    void on_exit() override { carriage_.hold(); }

private:
    // 诊断日志：业务日志不受 should_log_lifecycle() 影响，现场调试需要有输出
    template <typename... Args>
    void log(const char* fmt, Args... args) {
        if (runtime_context().logger != nullptr) {
            RCLCPP_INFO(*runtime_context().logger, fmt, args...);
        }
    }

    NewCarriageResource& carriage_;
    double up_velocity_;
    double back_off_velocity_;
    double back_off_distance_;
    uint64_t timeout_ticks_;
    int phase_{0};
    double hit_angle_{0.0};
};

// ── 标零：把当前位置设为 0 点 ────────────────────────────────────────────────
// 与 BeltCalibrateAction 完全对称。硬件层按上升沿生效，打一拍再放开。
class CarriageCalibrateAction : public manager::IAction {
public:
    explicit CarriageCalibrateAction(std::string name, NewCarriageResource& carriage)
        : IAction(std::move(name))
        , carriage_(carriage) {}

    void on_enter() override {
        carriage_.hold();
        carriage_.calibrate(true);
    }

    manager::ActionStatus update() override {
        if (elapsed_ticks() >= kCalibrateTicks)
            return manager::ActionStatus::SUCCESS;
        return manager::ActionStatus::RUNNING;
    }

    void on_exit() override {
        carriage_.calibrate(false);
        carriage_.hold();
    }

    void on_failure() override {
        carriage_.calibrate(false);
        carriage_.hold();
    }

private:
    static constexpr uint64_t kCalibrateTicks = 5;

    NewCarriageResource& carriage_;
};

// ── 走到绝对角度（P 控制，容差内成功）───────────────────────────────────────
class CarriageGotoAction : public manager::IAction {
public:
    CarriageGotoAction(
        std::string name, NewCarriageResource& carriage, double target, double gain,
        double max_velocity, double tolerance, uint64_t timeout_ticks)
        : IAction(std::move(name))
        , carriage_(carriage)
        , target_(target)
        , gain_(gain)
        , max_velocity_(std::abs(max_velocity))
        , tolerance_(std::abs(tolerance))
        , timeout_ticks_(timeout_ticks) {}

    manager::ActionStatus update() override {
        const double err = target_ - carriage_.angle();

        if (std::abs(err) <= tolerance_) {
            carriage_.hold();
            return manager::ActionStatus::SUCCESS;
        }
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_) {
            carriage_.hold();
            return fail(manager::ActionFailureReason::TIMEOUT);
        }

        carriage_.set_velocity(std::clamp(err * gain_, -max_velocity_, max_velocity_));
        return manager::ActionStatus::RUNNING;
    }

    void on_exit() override { carriage_.hold(); }

private:
    NewCarriageResource& carriage_;
    double target_;
    double gain_;
    double max_velocity_;
    double tolerance_;
    uint64_t timeout_ticks_;
};

// ── 参数非法时的占位动作 ───────────────────────────────────────────────────
// 用于「任务定义阶段就能发现」的错误（例如飞镖索引越界）。
// 不抛异常（构造期抛会在组件加载阶段炸掉整个进程），而是让任务跑第一帧就失败，
// 由 Manager 正常报错停机 —— 错误一定被观察到，不会静默跑错位置。
class ImmediateFailureAction : public manager::IAction {
public:
    explicit ImmediateFailureAction(std::string name)
        : IAction(std::move(name)) {}

    manager::ActionStatus update() override {
        return fail(manager::ActionFailureReason::INVALID_INPUT);
    }
};

} // namespace rmcs_dart_guidance::v2
