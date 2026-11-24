#pragma once

#include <cstdint>

#include <rmcs_dart_guidance/msg/mechanism_status.hpp>

namespace rmcs_dart_guidance::msg {

// 同步带上层动作类型（仅描述通用物理能力，不含具体业务阶段名）
enum class BeltActionType : uint8_t {
    IDLE = 0,
    RELEASE = 1,          // 放手（全写 NaN，由弹簧推回顶部零点）
    HOLD = 2,             // 零速保持
    HOME_TO_STALL = 3,    // 上推至机械顶堵转（归零力矩）
    RETURN_TO_STALL = 4,  // 空载上推回顶至堵转
    MOVE_REL = 5,         // 从当前角起相对位移（走梯形规划）
    CALIBRATE = 6,        // 标零脉冲（维持数 tick）
    PULL_TO_TRIGGER = 7,  // 分段下拉：低速过碰撞点 → 不断速加速 → 到底前减速
};

struct BeltActionCommand {
    uint32_t seq{0};                       // 单调递增订单号，防止连发同指令状态误判
    BeltActionType type{BeltActionType::IDLE};
    double distance{0.0};                  // MOVE_REL / PULL_TO_TRIGGER 有符号目标位移（rad）
    double velocity{0.0};                  // >0 时覆盖组件默认速度；<=0 用组件 yaml 默认值
    double torque_limit{0.0};              // >0 时覆盖组件默认力矩上限；<=0 用组件 yaml 默认值
    double approach_distance{0.0};         // >0 时覆盖碰撞切换距离（rad）
    double approach_velocity{0.0};         // >0 时覆盖低速靠近速度（rad/s）
};

// 弹舱上层动作类型
enum class CarriageActionType : uint8_t {
    IDLE = 0,
    HOLD = 1,       // 零速刹停
    HOME = 2,       // 撞微动开关 → 反向退离
    GOTO = 3,       // P 控转到目标绝对角度
    CALIBRATE = 4,  // 标零脉冲
};

struct CarriageActionCommand {
    uint32_t seq{0};
    CarriageActionType type{CarriageActionType::IDLE};
    double target_angle{0.0};              // GOTO 目标绝对角度（rad）
};

// 机构执行组件回执（ack_seq 必须等于下发的 seq 才代表对应命令的状态）
struct ActionFeedback {
    uint32_t ack_seq{0};
    MechanismStatus status{MechanismStatus::IDLE};
};

} // namespace rmcs_dart_guidance::msg
