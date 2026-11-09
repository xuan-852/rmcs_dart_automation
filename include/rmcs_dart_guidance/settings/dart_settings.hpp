#pragma once

#include <array>
#include <cstddef>

namespace rmcs_dart_guidance::v2 {

// ─────────────────────────────────────────────────────────────────────────────
// DartSettings
//   新镖架的全部可标定参数。
//
//   每发飞镖一组独立停位（上场允许 4~8 发，默认按 8 槽预留）：
//   改装数只需改 dart_count + 填对应槽位，代码一行不动。
//
//   速度/力矩/超时等工程参数也放这里，因为归零与到位的判断现在由上位机做，
//   下位机 controller 不含这些量。
// ─────────────────────────────────────────────────────────────────────────────
struct DartSettings {
    static constexpr int kMaxSlots = 8;

    // ── 飞镖 ────────────────────────────────────────────────────────────────
    int dart_count{0};
    std::array<double, kMaxSlots> dart_carriage_staypoints{};

    // ── 同步带 ──────────────────────────────────────────────────────────────
    double belt_trigger_distance{0.0};   // 顶部 → 扳机位的相对位移（实测）
    double belt_home_velocity{5.0};      // 归零上推速度
    double belt_down_velocity{5.0};      // 带载下降速度
    double belt_up_velocity{8.0};        // 上升速度
    double homing_torque_limit{3.0};     // 归零力矩上限（要够大才能真的顶住）
    double belt_load_torque_limit{6.0};  // 带载力矩上限
    double belt_unload_torque_limit{2.0};// 空载力矩上限

    // ── 弹舱 ────────────────────────────────────────────────────────────────
    double carriage_home_velocity{5.0};       // 归零向上撞开关速度
    double carriage_back_off_velocity{2.0};   // 开关触发后的退离速度
    double carriage_back_off_distance{0.1};   // 退离距离（退离后即零位）
    double carriage_goto_velocity{20.0};      // 定位速度上限
    double carriage_goto_gain{10.0};          // P 增益
    double carriage_goto_tolerance{0.02};     // 到位置容差

    bool configured() const {
        return dart_count > 0 && dart_count <= kMaxSlots && belt_trigger_distance > 0.0;
    }

    // 越界返回 false：由调用方决定失败，绝不静默取最后一发
    bool staypoint(int index, double& out) const {
        if (index < 0 || index >= dart_count)
            return false;
        out = dart_carriage_staypoints[static_cast<std::size_t>(index)];
        return true;
    }
};

} // namespace rmcs_dart_guidance::v2
