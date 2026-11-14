#pragma once

#include <rmcs_dart_guidance/new_dart/new_dart_task_builder.hpp>
#include <rmcs_dart_guidance/new_dart/new_dart_tasks.hpp>

#include <memory>
#include <string>
#include <string_view>

namespace rmcs_dart_guidance::v2::tasks {

// ═════════════════════════════════════════════════════════════════════════════
// 任务表
//
// 系统里有哪些任务，看这一屏就够了。
// 加任务：写一个 make_xxx_task 函数 → 在这张表里加一行。
// 分发逻辑（DartAgent）只查这张表，不认识任何具体任务。
// ═════════════════════════════════════════════════════════════════════════════
using TaskFactory = std::shared_ptr<manager::Task> (*)(TaskContext&);

struct TaskEntry {
    std::string_view name;          // 命令名（上位机下发的字符串）
    std::string_view description;   // 人读描述
    TaskFactory factory;
};

inline constexpr TaskEntry kAll[] = {
    {"dart-init", "初始化（同步带 + 弹舱归零）", &make_init_task},
    {"dart-launch-prepare", "发射准备（供弹 + 扳机就位）", &make_launch_prepare_task},
    {"dart-fire", "发射（松扳机弹出飞镖）", &make_fire_task},
    {"dart-launch-cancel", "取消发射（回顶待命）", &make_launch_cancel_task},
};

inline const TaskEntry* find(std::string_view cmd) {
    for (const auto& entry : kAll) {
        if (entry.name == cmd)
            return &entry;
    }
    return nullptr;
}

} // namespace rmcs_dart_guidance::v2::tasks
