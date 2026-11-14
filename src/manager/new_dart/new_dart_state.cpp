#include <rmcs_dart_guidance/new_dart/new_dart_state.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// 各状态的行为
//
// 顺序与 kAgentStateTable 一致，读这一份就知道每个状态干什么。
// 所有对持有者的操作都通过 ops 转发，本文件不依赖任何具体组件。
// ═════════════════════════════════════════════════════════════════════════════

// ── IDLE：队列空，等命令 ──────────────────────────────────────────────────
static void idle_enter(const AgentOps& ops) { ops.enter_idle(ops.ctx); }

static void idle_tick(const AgentOps& ops, bool both_down) {
    if (both_down) {
        ops.transition_to(ops.ctx, AgentState::kEmergency);
        return;
    }
    // 有任务开始跑才离开 IDLE
    if (ops.pump_tasks(ops.ctx))
        ops.transition_to(ops.ctx, AgentState::kRunning);
}

// ── RUNNING：有任务在跑 ───────────────────────────────────────────────────
static void running_enter(const AgentOps& ops) { (void)ops; }

static void running_tick(const AgentOps& ops, bool both_down) {
    if (both_down) {
        ops.transition_to(ops.ctx, AgentState::kEmergency);
        return;
    }
    ops.pump_tasks(ops.ctx);  // 内部失败时由 DartAgent 转到 kFailed
}

// ── EMERGENCY：双下锁存 ───────────────────────────────────────────────────
// 硬件层已经把「双下」定义成全电机 NaN 失能，但硬件层只是不再执行力矩，
// 它不知道上位机还有任务在跑。如果上位机不停，队列会继续推进，
// 一旦拨杆离开双下，机构会突然执行积压的、早就过时的目标。
//
// 注意：上位机这一层永远发有限值（零速 + 力矩限幅），NaN 只由硬件层决定。
static void emergency_enter(const AgentOps& ops) {
    ops.abort_all(ops.ctx, "E-STOP: both switches DOWN");
}

static void emergency_tick(const AgentOps& ops, bool both_down) {
    if (!both_down)
        ops.transition_to(ops.ctx, AgentState::kIdle);
}

// ── FAILED：任务失败锁存 ──────────────────────────────────────────────────
// 机构已停、队列已空。唯一出口是双下 → 急停 → 松开 → 空闲
static void failed_enter(const AgentOps& ops) { ops.abort_all(ops.ctx, nullptr); }

static void failed_tick(const AgentOps& ops, bool both_down) {
    if (both_down)
        ops.transition_to(ops.ctx, AgentState::kEmergency);
}

// ── 状态表 ────────────────────────────────────────────────────────────────
const AgentStateEntry kAgentStateTable[] = {
    {AgentState::kIdle, "IDLE", &idle_enter, &idle_tick},
    {AgentState::kRunning, "RUNNING", &running_enter, &running_tick},
    {AgentState::kEmergency, "EMERGENCY", &emergency_enter, &emergency_tick},
    {AgentState::kFailed, "FAILED", &failed_enter, &failed_tick},
};

const std::size_t kAgentStateCount = sizeof(kAgentStateTable) / sizeof(kAgentStateTable[0]);

const AgentStateEntry& agent_state_entry(AgentState state) {
    for (std::size_t i = 0; i < kAgentStateCount; ++i)
        if (kAgentStateTable[i].state == state)
            return kAgentStateTable[i];
    return kAgentStateTable[0];  // 不可达
}

} // namespace rmcs_dart_guidance::v2
