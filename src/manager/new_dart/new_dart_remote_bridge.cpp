#include <string>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

namespace rmcs_dart_guidance::v2 {

// ═════════════════════════════════════════════════════════════════════════════
// DartAgentRemoteBridge
//   新镖架的遥控器桥：把 DR16 拨杆翻成 /dart/manager/command 的字符串。
//
//   老镖架的 manager::RemoteCommandBridge 服务的是老任务体系（含裁判系统、
//   云台、底盘电平），语义和新镖架冲突 —— 它把「左 UP」当手动抑制，
//   而新镖架的手动档正是「左 UP + 右 MIDDLE」。所以这里独立写一份，
//   只服务 DartAgent，两套互不影响。
//
//   键位（照抄老镖架语义，老队员零学习成本）：
//
//     左 UP    + 右 MIDDLE          手动        ← 摇杆直驱，抑制本桥全部命令
//     左 MIDDLE + 右 MIDDLE         待命
//     左 MIDDLE + 右 DOWN           发射准备 ⇄ 取消准备（toggle）
//     左 MIDDLE + 右 UP             发射
//     左 DOWN → MIDDLE              归零（dart-init）
//     左右双下                      静默 —— 这是硬件层的全电机 NaN 急停，
//                                             本桥绝不占用，见下方说明
//
//   ⚠️ 双下为什么必须静默：
//     dart_new_launcher.cpp 的 update_motor_enable() 已经把「双下」定义为
//     全电机 NaN 失能（安全态）。如果桥也把双下翻成命令，会出现两个问题：
//       ① 安全态被当成功能键，操作员无法确定当前是「急停」还是「归零」；
//       ② 归零过程需要持续写目标值，与 NaN 失能直接冲突。
//     所以归零改用老镖架的「左 DOWN → MIDDLE」边沿：
//       双下（急停）→ 松开左拨杆到 MIDDLE（这条边沿）→ 自动归零
//     和 dart_manager.cpp 的 recover() 语义完全一致。
//
//   所有触发都是「边沿」：拨杆到位的那一 tick 发一次命令，然后保持静默。
//   命令是电平语义（DartAgent 靠「非空 → 与上次不同 → 入队」去重），
//   所以静默期必须显式写空串。
// ═════════════════════════════════════════════════════════════════════════════
class DartAgentRemoteBridge
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    DartAgentRemoteBridge()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true))
        , logger_(get_logger()) {
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
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/left\". Set to UNKNOWN.");
        }
        if (!switch_right_.ready()) {
            switch_right_.make_and_bind_directly(rmcs_msgs::Switch::UNKNOWN);
            RCLCPP_WARN(logger_, "Failed to fetch \"/remote/switch/right\". Set to UNKNOWN.");
        }

        RCLCPP_INFO(logger_, "[DartAgentRemoteBridge] 键位：");
        RCLCPP_INFO(logger_, "  manual  : left=UP         right=MID");
        RCLCPP_INFO(logger_, "  prepare : left=MID        right=MID->DOWN (toggle cancel)");
        RCLCPP_INFO(logger_, "  fire    : left=MID        right=MID->UP");
        RCLCPP_INFO(logger_, "  init    : left=DOWN->MID  (老镖架 recover 语义)");
        RCLCPP_INFO(logger_, "  双下     : 静默 (硬件层全电机 NaN 急停)");
    }

    void update() override {
        using rmcs_msgs::Switch;

        // 默认静默：DartAgent 把空串当作「当前无命令」
        *command_output_ = std::string{};

        const auto left = *switch_left_;
        const auto right = *switch_right_;

        // 拨杆无效（接收机失联）：不产生命令，并清掉边沿记忆，
        // 避免恢复瞬间把失联期间的状态跳变误判成一次触发。
        if (left == Switch::UNKNOWN || right == Switch::UNKNOWN) {
            remember(left, right);
            return;
        }

        // 上电第一个有效 tick：只记住状态，不产生命令。
        // 否则「开机时拨杆正好在 MID+DOWN」会被当成一次准备触发。
        if (prev_left_ == Switch::UNKNOWN || prev_right_ == Switch::UNKNOWN) {
            remember(left, right);
            return;
        }

        // ── 手动档：摇杆接管，抑制全部任务命令 ──────────────────────────────
        if (left == Switch::UP && right == Switch::MIDDLE) {
            prepare_armed_ = false;
            remember(left, right);
            return;
        }

        // ── 双下：硬件层的全电机 NaN 急停，本桥静默，绝不占用 ────────────────
        // 必须在所有命令判断之前 return，保证急停态下不会漏出任何任务命令。
        if (left == Switch::DOWN && right == Switch::DOWN) {
            prepare_armed_ = false;
            remember(left, right);
            return;
        }

        // ── 归零：左 DOWN → MIDDLE（老镖架 recover 语义）─────────────────────
        // 只管左边这根：双下急停后松左拨杆到 MIDDLE，就会走到这里自动归零。
        // 右边拨杆状态不参与判断，所以「双下 → 松左」和「左下上右」都能触发，
        // 和老镖架 detect_recover_transition() 完全一致。
        if (left == Switch::MIDDLE && prev_left_ == Switch::DOWN) {
            emit("dart-init");
            prepare_armed_ = false;
            remember(left, right);
            return;
        }

        // ── 发射准备 ⇄ 取消准备：左 MID，右 MIDLE→DOWN ───────────────────────
        if (left == Switch::MIDDLE && prev_left_ == Switch::MIDDLE
            && right == Switch::DOWN && prev_right_ != Switch::DOWN) {
            if (prepare_armed_) {
                emit("dart-launch-cancel");
                prepare_armed_ = false;
            } else {
                emit("dart-launch-prepare");
                prepare_armed_ = true;
            }
            remember(left, right);
            return;
        }

        // ── 发射：左 MID，右 MIDDLE→UP ──────────────────────────────────────
        if (left == Switch::MIDDLE && prev_left_ == Switch::MIDDLE
            && right == Switch::UP && prev_right_ != Switch::UP) {
            emit("dart-fire");
            prepare_armed_ = false;
            remember(left, right);
            return;
        }

        remember(left, right);
    }

private:
    void emit(const char* cmd) {
        *command_output_ = std::string{cmd};
        RCLCPP_INFO(logger_, "[DartAgentRemoteBridge] -> \"%s\"", cmd);
    }

    void remember(rmcs_msgs::Switch left, rmcs_msgs::Switch right) {
        prev_left_ = left;
        prev_right_ = right;
    }

    rclcpp::Logger logger_;

    InputInterface<rmcs_msgs::Switch> switch_left_;
    InputInterface<rmcs_msgs::Switch> switch_right_;
    OutputInterface<std::string> command_output_;

    rmcs_msgs::Switch prev_left_{rmcs_msgs::Switch::UNKNOWN};
    rmcs_msgs::Switch prev_right_{rmcs_msgs::Switch::UNKNOWN};

    // 「准备」是否已下发，决定下一次右 DOWN 是准备还是取消
    bool prepare_armed_{false};
};

} // namespace rmcs_dart_guidance::v2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    rmcs_dart_guidance::v2::DartAgentRemoteBridge, rmcs_executor::Component)
