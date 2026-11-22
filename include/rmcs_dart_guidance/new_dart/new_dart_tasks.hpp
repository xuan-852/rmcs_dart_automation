#pragma once

#include "manager/runtime/action.hpp"
#include "manager/runtime/manager_types.hpp"
#include "manager/runtime/task.hpp"

#include <rmcs_dart_guidance/action/configuration_guard_action.hpp>
#include <rmcs_dart_guidance/action/delay_action.hpp>
#include <rmcs_dart_guidance/action/fire_count_increment_action.hpp>
#include <rmcs_dart_guidance/msg/new_dart_action.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/logging.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

inline manager::ActionStatus check_action_feedback(
    const rmcs_dart_guidance::msg::ActionFeedback& fb, uint32_t req_seq) {
    using MechStatus = rmcs_dart_guidance::msg::MechanismStatus;
    if (fb.ack_seq != req_seq)
        return manager::ActionStatus::RUNNING;
    if (fb.status == MechStatus::SUCCEEDED)
        return manager::ActionStatus::SUCCESS;
    if (fb.status == MechStatus::FAILED || fb.status == MechStatus::ABORTED)
        return manager::ActionStatus::FAILURE;
    return manager::ActionStatus::RUNNING;
}

// ═════════════════════════════════════════════════════════════════════════════
// 1. 新镖架转接资源（对接 BeltActionComponent / CarriageActionComponent）
// ═════════════════════════════════════════════════════════════════════════════

class BeltActionResource {
    using Cmd = rmcs_dart_guidance::msg::BeltActionCommand;
    using Type = rmcs_dart_guidance::msg::BeltActionType;
    using Feedback = rmcs_dart_guidance::msg::ActionFeedback;

public:
    BeltActionResource(
        rmcs_executor::Component& status_comp, rmcs_executor::Component& cmd_comp,
        const std::string& prefix = "/dart/belt_action") {
        cmd_comp.register_output(prefix + "/command", command_, Cmd{});
        status_comp.register_input(prefix + "/feedback", feedback_, false);
    }

    BeltActionResource(const BeltActionResource&) = delete;
    BeltActionResource& operator=(const BeltActionResource&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!feedback_.ready()) {
            feedback_.make_and_bind_directly(Feedback{});
            RCLCPP_WARN(logger, "Failed to fetch \"/dart/belt_action/feedback\". Set to IDLE.");
        }
    }

    uint32_t request(Type type, double distance = 0.0) {
        *command_ = Cmd{++seq_, type, distance, 0.0, 0.0};
        return seq_;
    }

    // 故障/急停：同步带放手（NaN）让弹簧推回顶部零点
    void safe_stop() { request(Type::RELEASE); }

    manager::ActionStatus check(uint32_t req_seq) const {
        return check_action_feedback(*feedback_, req_seq);
    }

private:
    uint32_t seq_{0};
    rmcs_executor::Component::OutputInterface<Cmd> command_;
    rmcs_executor::Component::InputInterface<Feedback> feedback_;
};

class CarriageActionResource {
    using Cmd = rmcs_dart_guidance::msg::CarriageActionCommand;
    using Type = rmcs_dart_guidance::msg::CarriageActionType;
    using Feedback = rmcs_dart_guidance::msg::ActionFeedback;

public:
    CarriageActionResource(
        rmcs_executor::Component& status_comp, rmcs_executor::Component& cmd_comp,
        const std::string& prefix = "/dart/carriage_action") {
        cmd_comp.register_output(prefix + "/command", command_, Cmd{});
        status_comp.register_input(prefix + "/feedback", feedback_, false);
    }

    CarriageActionResource(const CarriageActionResource&) = delete;
    CarriageActionResource& operator=(const CarriageActionResource&) = delete;

    void bind_optional(const rclcpp::Logger& logger) {
        if (!feedback_.ready()) {
            feedback_.make_and_bind_directly(Feedback{});
            RCLCPP_WARN(
                logger, "Failed to fetch \"/dart/carriage_action/feedback\". Set to IDLE.");
        }
    }

    uint32_t request(Type type, double target_angle = 0.0) {
        *command_ = Cmd{++seq_, type, target_angle};
        return seq_;
    }

    // 故障/急停：旋转货仓保持零速停在当前角度
    void safe_stop() { request(Type::HOLD); }

    manager::ActionStatus check(uint32_t req_seq) const {
        return check_action_feedback(*feedback_, req_seq);
    }

private:
    uint32_t seq_{0};
    rmcs_executor::Component::OutputInterface<Cmd> command_;
    rmcs_executor::Component::InputInterface<Feedback> feedback_;
};

// ═════════════════════════════════════════════════════════════════════════════
// 2. 通用带序号发令 Action（对标 action/mechanism_command_action.hpp）
// ═════════════════════════════════════════════════════════════════════════════

template <typename Resource, typename CommandType>
class SeqCommandAction final : public manager::IAction {
public:
    SeqCommandAction(
        std::string name, Resource& resource, CommandType cmd, uint64_t timeout_ticks,
        double param = 0.0)
        : IAction(std::move(name))
        , resource_(resource)
        , cmd_(cmd)
        , timeout_ticks_(timeout_ticks)
        , param_(param) {}

    void on_enter() override { req_seq_ = resource_.request(cmd_, param_); }

    manager::ActionStatus update() override {
        const auto status = resource_.check(req_seq_);
        if (status == manager::ActionStatus::SUCCESS)
            return manager::ActionStatus::SUCCESS;
        if (status == manager::ActionStatus::FAILURE)
            return fail(manager::ActionFailureReason::DEPENDENCY_FAILURE);
        if (timeout_ticks_ > 0 && elapsed_ticks() >= timeout_ticks_)
            return fail(manager::ActionFailureReason::TIMEOUT);
        return manager::ActionStatus::RUNNING;
    }

    void on_failure() override { resource_.safe_stop(); }

    void on_cancel(manager::ActionCancelReason reason) override {
        if (reason != manager::ActionCancelReason::NORMAL_COMPLETION)
            resource_.safe_stop();
    }

private:
    Resource& resource_;
    CommandType cmd_;
    uint64_t timeout_ticks_;
    double param_;
    uint32_t req_seq_{0};
};

// ═════════════════════════════════════════════════════════════════════════════
// 3. 任务剧本与构建工厂（复用老镖架 manager::Task / Delay / Guard / FireCount）
// ═════════════════════════════════════════════════════════════════════════════

struct NewDartSettings {
    static constexpr int kMaxSlots = 8;

    int dart_count{0};
    std::array<double, kMaxSlots> staypoints{};
    double belt_trigger_distance{0.0};

    bool configured() const {
        return dart_count > 0 && dart_count <= kMaxSlots && belt_trigger_distance > 0.0;
    }
};

inline std::shared_ptr<manager::Task> make_task(
    const std::string& cmd, BeltActionResource& belt, CarriageActionResource& carriage,
    manager::ManagerRuntimeState& runtime_state, const NewDartSettings& settings) {
    using BeltType = rmcs_dart_guidance::msg::BeltActionType;
    using CarriageType = rmcs_dart_guidance::msg::CarriageActionType;
    using BeltAction = SeqCommandAction<BeltActionResource, BeltType>;
    using CarriageAction = SeqCommandAction<CarriageActionResource, CarriageType>;

    constexpr uint64_t kHomeTimeoutTicks = 500000;
    constexpr uint64_t kMechanismTimeoutTicks = 300000;
    constexpr uint64_t kCalibrateTimeoutTicks = 1000;

    if (cmd == "dart-init") {
        auto task = std::make_shared<manager::Task>("dart-init", "初始化");
        task->then(std::make_shared<BeltAction>(
            "belt_home", belt, BeltType::HOME_TO_STALL, kHomeTimeoutTicks));
        task->then(std::make_shared<BeltAction>(
            "belt_calibrate", belt, BeltType::CALIBRATE, kCalibrateTimeoutTicks));
        task->then(std::make_shared<CarriageAction>(
            "carriage_home", carriage, CarriageType::HOME, kHomeTimeoutTicks));
        task->then(std::make_shared<CarriageAction>(
            "carriage_calibrate", carriage, CarriageType::CALIBRATE, kCalibrateTimeoutTicks));
        return task;
    }

    if (cmd == "dart-launch-prepare") {
        auto task = std::make_shared<manager::Task>("dart-launch-prepare", "发射准备");
        const int idx = static_cast<int>(runtime_state.fire_count);
        const bool slot_ok = settings.configured() && idx >= 0 && idx < settings.dart_count;
        const double target_angle =
            slot_ok ? settings.staypoints[static_cast<std::size_t>(idx)] : 0.0;

        task->then(std::make_shared<manager::ConfigurationGuardAction>(
            "settings_and_slot_configured", slot_ok));
        task->then(std::make_shared<CarriageAction>(
            "carriage_goto_current", carriage, CarriageType::GOTO, kMechanismTimeoutTicks,
            target_angle));
        task->then(std::make_shared<BeltAction>(
            "belt_down_to_trigger", belt, BeltType::MOVE_REL, kMechanismTimeoutTicks,
            -std::abs(settings.belt_trigger_distance)));
        task->then(std::make_shared<manager::DelayAction>("settle_delay", 200));
        // TODO: trigger_lock —— 扳机舵机接入后替换
        task->then(std::make_shared<manager::DelayAction>("trigger_lock_placeholder", 200));
        task->then(std::make_shared<BeltAction>(
            "belt_up_to_top", belt, BeltType::RETURN_TO_STALL, kMechanismTimeoutTicks));
        return task;
    }

    if (cmd == "dart-fire") {
        auto task = std::make_shared<manager::Task>("dart-fire", "发射");
        const bool can_fire = settings.dart_count <= 0
                           || static_cast<int>(runtime_state.fire_count) < settings.dart_count;

        task->then(std::make_shared<manager::ConfigurationGuardAction>("darts_remaining", can_fire));
        // TODO: trigger_free —— 扳机舵机接入后替换
        task->then(std::make_shared<manager::DelayAction>("trigger_free_placeholder", 500));
        task->then(std::make_shared<manager::FireCountIncrementAction>(
            "fire_count_increment", runtime_state.fire_count));
        return task;
    }

    if (cmd == "dart-launch-cancel") {
        auto task = std::make_shared<manager::Task>("dart-launch-cancel", "取消发射");
        task->then(std::make_shared<BeltAction>(
            "belt_up_to_top", belt, BeltType::RETURN_TO_STALL, kMechanismTimeoutTicks));
        return task;
    }

    return nullptr;
}

} // namespace rmcs_dart_guidance::v2
