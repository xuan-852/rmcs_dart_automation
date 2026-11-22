#include <optional>
#include <string>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgentRemoteBridge
//   新镖架遥控器桥：将 DR16 拨杆边沿按规则表翻为 /dart/manager/command。
//
//   键位：
//     左 UP    + 右 MID           手动（本桥静默，DartAgent 同时中止运行中任务）
//     左右双下                    静默（硬件层全电机 NaN 急停 + DartAgent 锁 FAILED）
//     左 DOWN → MID               recover（解锁 FAILED + 清发射计数 + 自动归零）
//     左 MID  + 右 MID → DOWN     发射准备 ⇄ 取消准备（toggle）
//     左 MID  + 右 MID → UP       发射
// ═════════════════════════════════════════════════════════════════════════════
class DartAgentRemoteBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAgentRemoteBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/remote/switch/left", switch_left_, false);
        register_input("/remote/switch/right", switch_right_, false);

        register_output("/dart/manager/command", command_output_, std::string{});
    }

    DartAgentRemoteBridge(const DartAgentRemoteBridge&) = delete;
    DartAgentRemoteBridge& operator=(const DartAgentRemoteBridge&) = delete;
    DartAgentRemoteBridge(DartAgentRemoteBridge&&) = delete;
    DartAgentRemoteBridge& operator=(DartAgentRemoteBridge&&) = delete;

    ~DartAgentRemoteBridge() override = default;

    void before_updating() override {
        if (!switch_left_.ready()) {
            switch_left_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(get_logger(), "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }

        RCLCPP_INFO(get_logger(), "[DartAgentRemoteBridge] 键位：");
        RCLCPP_INFO(get_logger(), "  manual  : left=UP         right=MID");
        RCLCPP_INFO(get_logger(), "  recover : left=DOWN->MID  (解锁 FAILED 并触发 dart-init)");
        RCLCPP_INFO(get_logger(), "  prepare : left=MID        right=MID->DOWN (toggle cancel)");
        RCLCPP_INFO(get_logger(), "  fire    : left=MID        right=MID->UP");
        RCLCPP_INFO(get_logger(), "  双下     : 静默 (硬件层 NaN 急停 + DartAgent 锁 FAILED)");
    }

    void update() override {
        using rmcs_msgs::Switch;

        *command_output_ = std::string{};

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        // 接收机断联、上电首帧、手动档或双下急停：不触发任务边沿，统一走末尾记录
        const bool suppressed = left == Switch::UNKNOWN || right == Switch::UNKNOWN
                             || prev_left_ == Switch::UNKNOWN || prev_right_ == Switch::UNKNOWN
                             || (left == Switch::UP && right == Switch::MIDDLE)
                             || (left == Switch::DOWN && right == Switch::DOWN);
        if (suppressed) {
            prepare_armed_ = false;
        } else {
            for (const auto& rule : kRules) {
                const bool hit = (!rule.prev_left || *rule.prev_left == prev_left_)
                              && (!rule.left || *rule.left == left)
                              && (!rule.prev_right || *rule.prev_right == prev_right_)
                              && (!rule.right || *rule.right == right);
                if (!hit)
                    continue;

                prepare_armed_ = rule.toggles_prepare ? !prepare_armed_ : false;
                const char* cmd = (rule.toggles_prepare && !prepare_armed_) ? rule.cmd_off : rule.cmd_on;
                *command_output_ = std::string{cmd};
                RCLCPP_INFO(get_logger(), "[DartAgentRemoteBridge] -> \"%s\"", cmd);
                break;
            }
        }

        prev_left_ = left;
        prev_right_ = right;
    }

private:
    struct SwitchRule {
        std::optional<rmcs_msgs::Switch> prev_left;
        std::optional<rmcs_msgs::Switch> left;
        std::optional<rmcs_msgs::Switch> prev_right;
        std::optional<rmcs_msgs::Switch> right;
        bool toggles_prepare;
        const char* cmd_on;
        const char* cmd_off;
    };

    static constexpr SwitchRule kRules[] = {
        {rmcs_msgs::Switch::DOWN,   rmcs_msgs::Switch::MIDDLE, std::nullopt,               std::nullopt,               false, "recover",             nullptr},
        {rmcs_msgs::Switch::MIDDLE, rmcs_msgs::Switch::MIDDLE, rmcs_msgs::Switch::MIDDLE,  rmcs_msgs::Switch::DOWN,    true,  "dart-launch-prepare", "dart-launch-cancel"},
        {rmcs_msgs::Switch::MIDDLE, rmcs_msgs::Switch::MIDDLE, rmcs_msgs::Switch::MIDDLE,  rmcs_msgs::Switch::UP,      false, "dart-fire",           nullptr},
    };

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};
    bool prepare_armed_{false};

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;

    OutputInterface<std::string> command_output_;
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::v2::DartAgentRemoteBridge, rmcs_executor::Component)
