#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgentTestDriver（调试用）
//   按时间顺序自动往 /dart/manager/command 打一串命令，用来在没有遥控器、
//   没有物理上位机的情况下验证任务表能否被正确查表、排队、推进。
//
//   yaml 里给一个字符串序列即可：
//     dart_agent_test_driver:
//       ros__parameters:
//         script: ["dart-init", "", "dart-launch-prepare", "", "dart-fire"]
//         hold_ticks: 500            # 每条命令保持多少 tick（0 = 立即切下一条）
//
//   仅用于台架调试，正式上场不要放进 components。
// ═════════════════════════════════════════════════════════════════════════════
class DartAgentTestDriver : public rmcs_executor::Component, public rclcpp::Node {
public:
    DartAgentTestDriver()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_output("/dart/manager/command", command_, std::string{});

        const auto script = get_parameter("script").as_string_array();
        for (const auto& item : script)
            script_.push_back(item);
        hold_ticks_ = get_parameter("hold_ticks").as_int();

        RCLCPP_INFO(
            logger_, "[TestDriver] %zu commands, hold_ticks=%ld", script_.size(),
            static_cast<long>(hold_ticks_));
    }

    void update() override {
        if (script_.empty())
            return;

        if (ticks_ >= hold_ticks_) {
            ticks_ = 0;
            *command_ = script_.front();
            script_.erase(script_.begin());
            RCLCPP_INFO(logger_, "[TestDriver] pushed \"%s\"", command_->c_str());
            return;
        }
        ++ticks_;
    }

private:
    rclcpp::Logger logger_;
    OutputInterface<std::string> command_;
    std::vector<std::string> script_;
    std::int64_t hold_ticks_{500};
    std::int64_t ticks_{0};
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::v2::DartAgentTestDriver, rmcs_executor::Component)
