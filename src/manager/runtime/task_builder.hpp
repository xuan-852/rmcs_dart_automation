#pragma once

#include "manager/runtime/action_sequence.hpp"
#include "manager/runtime/action_set.hpp"
#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"

#include <rmcs_dart_guidance/action/delay_action.hpp>
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/action/new_launcher_action.hpp>
#include <rmcs_dart_guidance/resource/new_launcher_resource.hpp>
#include <rmcs_dart_guidance/settings/dart_settings.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace rmcs_dart_guidance::v2 {

// 任务运行所需的全部外部依赖
struct TaskContext {
    NewBeltResource& belt;
    NewCarriageResource& carriage;
    DartSettings& settings;
    manager::ManagerRuntimeState& state;
};

// ─────────────────────────────────────────────────────────────────────────────
// TaskBuilder：任务书写用的链式语法糖
//
// 把「名字 / 资源 / 超时 / 速度」这些脚手架收进内部，任务里只写业务动作：
//   .delay(1000ms)
//   .belt_home()
//   .belt_down(1234.0)
//   .carriage_goto_dart(0)
//   .fire_count_inc();
//
// 每个动作自己推进状态、自己判断完成条件（见 new_launcher_action.hpp），
// 下位机 controller 只负责把目标转发给电机。
// ─────────────────────────────────────────────────────────────────────────────
class TaskBuilder : public manager::Task {
public:
    using Task = manager::Task;

    TaskBuilder(std::string name, std::string description, TaskContext& ctx)
        : Task(std::move(name), std::move(description))
        , ctx_(ctx) {}

    // ── 时序 ────────────────────────────────────────────────────────────────
    TaskBuilder& delay(std::chrono::milliseconds ms) {
        return add<manager::DelayAction>("delay", static_cast<uint64_t>(ms.count()));
    }

    // ── 同步带归零：上推堵转 → 标零 ───────────────────────────────────────────
    TaskBuilder& belt_home() {
        const auto& s = ctx_.settings;
        then(std::make_shared<BeltStallAction>(
            "belt_home_up", ctx_.belt, -s.belt_home_velocity, s.homing_torque_limit,
            kHomeTimeoutTicks));
        then(std::make_shared<BeltCalibrateAction>(
            "belt_home_zero", ctx_.belt, s.homing_torque_limit));
        return *this;
    }

    // ── 同步带相对位移 ──────────────────────────────────────────────────────
    TaskBuilder& belt_down(double distance) {
        return add<BeltMoveAction>(
            "belt_down", ctx_.belt, -ctx_.settings.belt_down_velocity, distance,
            ctx_.settings.belt_load_torque_limit, kMechanismTimeoutTicks);
    }

    TaskBuilder& belt_up(double distance) {
        return add<BeltMoveAction>(
            "belt_up", ctx_.belt, ctx_.settings.belt_up_velocity, distance,
            ctx_.settings.belt_load_torque_limit, kMechanismTimeoutTicks);
    }

    // 下降到扳机位（用 yaml 的 belt_trigger_distance，任务里不写数字）
    TaskBuilder& belt_down_to_trigger() {
        return belt_down(ctx_.settings.belt_trigger_distance);
    }

    // 空载回顶：一直上推直到堵转
    TaskBuilder& belt_up_to_stall() {
        return add<BeltStallAction>(
            "belt_up_to_stall", ctx_.belt, ctx_.settings.belt_up_velocity,
            ctx_.settings.belt_unload_torque_limit, kMechanismTimeoutTicks);
    }

    // 把当前位置标为 0 点
    TaskBuilder& belt_calibrate() {
        return add<BeltCalibrateAction>(
            "belt_calibrate", ctx_.belt, ctx_.settings.homing_torque_limit);
    }

    // ── 弹舱 ────────────────────────────────────────────────────────────────
    TaskBuilder& carriage_home() {
        const auto& s = ctx_.settings;
        return add<CarriageHomeAction>(
            "carriage_home", ctx_.carriage, s.carriage_home_velocity,
            s.carriage_back_off_velocity, s.carriage_back_off_distance, kHomeTimeoutTicks);
    }

    // 把当前位置标为 0 点（归零的最后一步，和 belt_calibrate 对称）
    TaskBuilder& carriage_calibrate() {
        return add<CarriageCalibrateAction>("carriage_calibrate", ctx_.carriage);
    }

    TaskBuilder& carriage_goto(double angle) {
        const auto& s = ctx_.settings;
        return add<CarriageGotoAction>(
            "carriage_goto", ctx_.carriage, angle, s.carriage_goto_gain,
            s.carriage_goto_velocity, s.carriage_goto_tolerance, kMechanismTimeoutTicks);
    }

    // 走到第 index 发的停位。索引越界 → 插一个必然失败的动作，绝不静默跑错位置。
    TaskBuilder& carriage_goto_dart(int index) {
        double angle = 0.0;
        if (!ctx_.settings.staypoint(index, angle))
            return add_invalid_index(index);
        return carriage_goto(angle);
    }

    // 走到「当前发射计数」对应的那一发（首发为 0）
    TaskBuilder& carriage_goto_current() {
        return carriage_goto_dart(static_cast<int>(ctx_.state.fire_count));
    }

    // ── 计数 ────────────────────────────────────────────────────────────────
    TaskBuilder& fire_count_inc() {
        return add<manager::FireCountIncrementAction>(
            "fire_count_increment", ctx_.state.fire_count);
    }

private:
    template <typename Action, typename... Args>
    TaskBuilder& add(const char* name, Args&&... args) {
        then(std::make_shared<Action>(name, std::forward<Args>(args)...));
        return *this;
    }

    // 飞镖索引越界：插一个立刻失败的动作，让任务跑起来时明确报错
    TaskBuilder& add_invalid_index(int index) {
        then(std::make_shared<ImmediateFailureAction>(
            "carriage_goto_dart:index_" + std::to_string(index) + "_out_of_range"));
        return *this;
    }

    TaskContext& ctx_;

    static constexpr uint64_t kMechanismTimeoutTicks = 300000;
    static constexpr uint64_t kHomeTimeoutTicks = 500000;
};

} // namespace rmcs_dart_guidance::v2
