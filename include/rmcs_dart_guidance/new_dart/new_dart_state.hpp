#pragma once

#include <cstddef>
#include <cstdint>

#include <rclcpp/logger.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// 任务代理的状态机框架
//
// 本文件不依赖任何具体组件，通过 AgentOps 函数表操作持有者。
//
// 每个状态自带两件事：进入做什么（enter）、每 tick 做什么（tick）。
// 状态转换走 ops.transition_to()，由持有者统一处理日志与 enter 回调。
//
// 加状态：加一个 enum 值 + 写 enter/tick 两个函数 + 在状态表加一行。
//
// ⚠️ enter() 里禁止调 ops.transition_to()，否则递归。
//    enter 只做「停机构 / 清状态」，状态转换全部写在 tick() 里。
// ═════════════════════════════════════════════════════════════════════════════

enum class AgentState : std::uint8_t {
    kIdle = 0,       // 队列空，等命令
    kRunning = 1,    // 有任务在跑
    kEmergency = 2,  // 双下锁存，队列已清、机构已停
    kFailed = 3,     // 任务失败锁存，队列已清、机构已停
};

// 状态机可以对持有者做的全部操作。持有者构造时填这张表。
//
// 只放「语义级」操作，不放实现细节：
//   transition_to  唯一的状态控制
//   enter_idle     进入空闲（清当前任务）
//   pump_tasks     收命令 + 推进队列，返回 true 表示有任务在跑
//   abort_all      停机构 + 清队列 + 报错（reason 为 nullptr 时不报）
//
// 日志、队列长度、错误字段这些实现细节由持有者内部处理，不暴露给状态机。
struct AgentOps {
    void* ctx;

    void (*transition_to)(void* ctx, AgentState next);
    void (*enter_idle)(void* ctx);
    bool (*pump_tasks)(void* ctx);
    void (*abort_all)(void* ctx, const char* reason);
};

struct AgentStateEntry {
    AgentState state;
    const char* name;
    void (*enter)(const AgentOps& ops);
    void (*tick)(const AgentOps& ops, bool both_down);
};

// 状态表定义在 dart_agent_state.cpp
extern const AgentStateEntry kAgentStateTable[];
extern const std::size_t kAgentStateCount;

const AgentStateEntry& agent_state_entry(AgentState state);

} // namespace rmcs_dart_guidance::v2
