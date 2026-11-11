#pragma once

#include "manager/runtime/task_builder.hpp"

#include <chrono>
#include <memory>
#include <string>

namespace rmcs_dart_guidance::v2::tasks {

using namespace std::chrono_literals;

// ═════════════════════════════════════════════════════════════════════════════
// 新镖架的全部任务
//
//  一个任务就是一个函数，从上往下读就是机构动作顺序。
//  每个动作自己判断完成条件（堵转 / 到位 / 超时），下位机只负责转发目标。
//  加任务 = 在这下面加一个函数 + 在 kAll 表里加一行，框架一行不用改。
// ═════════════════════════════════════════════════════════════════════════════

// ── dart-init：归零 ─────────────────────────────────────────────────────────
// 同步带上推堵转找顶；弹舱撞微动开关后退离。两者互不干涉，并行跑。
// 之后两侧各自标零，写法完全对称。
inline std::shared_ptr<manager::Task> make_init_task(TaskContext& ctx) {
    auto task = std::make_shared<TaskBuilder>("dart-init", "初始化", ctx);

    // 并行找机械零位
    // 速度符号约定：正 = 向上。belt_home_velocity 配正数即向上顶。
    auto seek = std::make_shared<manager::ActionSet>("init_seek_zero");
    seek->also(std::make_shared<BeltStallAction>(
        "belt_seek_top", ctx.belt, ctx.settings.belt_home_velocity,
        ctx.settings.homing_torque_limit, 500000));
    seek->also(std::make_shared<CarriageHomeAction>(
        "carriage_seek_switch", ctx.carriage, ctx.settings.carriage_home_velocity,
        ctx.settings.carriage_back_off_velocity, ctx.settings.carriage_back_off_distance,
        500000));
    task->then(seek);

    // 两侧都到位后再标零（标零必须在找零之后，否则 0 点漂）
    task->belt_calibrate();
    task->carriage_calibrate();

    return task;
}

// ── dart-launch-prepare：发射准备 ───────────────────────────────────────────
// 弹舱转到供弹位 → 同步带下降挂上飞镖 → 继续降到超过扳机位
// → 扳机升起挡住滑块 → 同步带空载回顶。
inline std::shared_ptr<manager::Task> make_launch_prepare_task(TaskContext& ctx) {
    auto task = std::make_shared<TaskBuilder>("dart-launch-prepare", "发射准备", ctx);

    task->carriage_goto_current();      // 转到当前这一发的停位
    task->belt_down_to_trigger();       // 带载下降到超过扳机位
    task->delay(200ms);                 // 等滑块停稳
    // TODO: trigger_lock() —— 扳机舵机尚未接入
    task->delay(200ms);
    task->belt_up_to_stall();           // 空载回顶

    return task;
}

// ── dart-fire：发射 ─────────────────────────────────────────────────────────
// 扳机放下 → 弹簧把滑块连同飞镖弹出 → 滑块自行回原位。
inline std::shared_ptr<manager::Task> make_fire_task(TaskContext& ctx) {
    auto task = std::make_shared<TaskBuilder>("dart-fire", "发射", ctx);

    // TODO: trigger_free() —— 扳机舵机尚未接入
    task->delay(500ms);
    task->fire_count_inc();

    return task;
}

// ── dart-launch-cancel：取消发射 ────────────────────────────────────────────
// 尚未定型，先占位：同步带回顶待命。
inline std::shared_ptr<manager::Task> make_launch_cancel_task(TaskContext& ctx) {
    auto task = std::make_shared<TaskBuilder>("dart-launch-cancel", "取消发射", ctx);

    task->belt_up_to_stall();

    return task;
}

} // namespace rmcs_dart_guidance::v2::tasks
