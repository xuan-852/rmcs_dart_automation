#include <cstdint>
#include <string>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartStickSim（仅台架验证用）
//   按脚本驱动 /remote/switch/left 与 /remote/switch/right，用来在没有真
//   遥控器的情况下验证 DartAgentRemoteBridge 的键位与边沿逻辑。
//
//   yaml:
//     script: ["MID,MID", "MID,DOWN", "MID,MID", "MID,UP", "DOWN,DOWN"]
//     hold_ticks: 300        # 每一步保持多少 tick
//
//   故意不放进正式 components。
// ═════════════════════════════════════════════════════════════════════════════
class DartStickSim : public rmcs_executor::Component, public rclcpp::Node {
public:
    DartStickSim()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
        register_output("/remote/switch/left", left_, rmcs_msgs::Switch::UNKNOWN);
        register_output("/remote/switch/right", right_, rmcs_msgs::Switch::UNKNOWN);

        const auto script = get_parameter("script").as_string_array();
        for (const auto& step : script) {
            const auto comma = step.find(',');
            if (comma == std::string::npos) {
                RCLCPP_ERROR(logger_, "[StickSim] bad step \"%s\"", step.c_str());
                continue;
            }
            steps_.push_back(
                {parse(step.substr(0, comma)), parse(step.substr(comma + 1))});
        }
        hold_ticks_ = get_parameter("hold_ticks").as_int();

        RCLCPP_INFO(
            logger_, "[StickSim] %zu steps, hold_ticks=%ld", steps_.size(),
            static_cast<long>(hold_ticks_));
    }

    void update() override {
        if (index_ >= steps_.size())
            return;

        // 日志必须在赋值前打，否则读日志的人会以为「打印的这一档触发了下一条」
        if (ticks_ == 0) {
            RCLCPP_INFO(
                logger_, "[StickSim] step %zu -> left=%s right=%s", index_,
                name(steps_[index_].first), name(steps_[index_].second));
        }

        *left_ = steps_[index_].first;
        *right_ = steps_[index_].second;

        if (++ticks_ < hold_ticks_)
            return;

        ticks_ = 0;
        ++index_;
    }

private:
    static const char* name(rmcs_msgs::Switch s) {
        switch (s) {
        case rmcs_msgs::Switch::UP: return "UP";
        case rmcs_msgs::Switch::DOWN: return "DOWN";
        case rmcs_msgs::Switch::MIDDLE: return "MID";
        default: return "?";
        }
    }

    static rmcs_msgs::Switch parse(const std::string& s) {
        if (s == "UP")
            return rmcs_msgs::Switch::UP;
        if (s == "DOWN")
            return rmcs_msgs::Switch::DOWN;
        if (s == "MID")
            return rmcs_msgs::Switch::MIDDLE;
        return rmcs_msgs::Switch::UNKNOWN;
    }

    rclcpp::Logger logger_;
    OutputInterface<rmcs_msgs::Switch> left_;
    OutputInterface<rmcs_msgs::Switch> right_;

    std::vector<std::pair<rmcs_msgs::Switch, rmcs_msgs::Switch>> steps_;
    std::size_t index_{0};
    std::int64_t hold_ticks_{300};
    std::int64_t ticks_{0};
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_dart_guidance::v2::DartStickSim, rmcs_executor::Component)
