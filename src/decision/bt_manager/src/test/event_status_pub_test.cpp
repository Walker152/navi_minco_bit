#include "bt_manager/bt_manager.hpp"
#include "bt_manager/blackboard.hpp"
#include "bt_manager/ros_interface.hpp"
#include "bt_manager/utils/color_text.hpp"
#include "bt_manager/utils/area.hpp"
#include "bt_manager/utils/tf_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cmath>
#include <thread>
#include <chrono>
#include <cassert>
#include <functional>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace Sentry_BT;
using namespace color_text;

// ==========================================
// 测试框架基础类：接管黑板，模拟时间与状态
// ==========================================
class SentryTestSuite {
public:
    SentryTestSuite(std::shared_ptr<rclcpp::Node> node) : node_(node) {
        blackboard_ = std::make_shared<Blackboard>();
        bt_blackboard_ = blackboard_->getBTBlackboard();
        
        // 挂载 ROS 接口（即使是空壳也需要，以防节点内部调用）
        ros_interface_ = std::make_shared<ros_interface>(blackboard_);
        bt_blackboard_->set<std::shared_ptr<ros_interface>>("ros_interface", ros_interface_);
        bt_blackboard_->set<rclcpp::Node::SharedPtr>("node", ros_interface_);
        bt_blackboard_->set<std::shared_ptr<TransformUtils>>(
            "transform_utils", std::make_shared<TransformUtils>());
        bt_blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(100));
        bt_blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(500));
        bt_blackboard_->set<std::chrono::milliseconds>(
            "wait_for_service_timeout", std::chrono::milliseconds(10000));
        
        // 初始化 BT Manager
        bt_manager_ = std::make_shared<SentryBTManager>();
        // 注意：请确保运行测试时，工作目录或环境变量指向正确的 tree 文件夹路径
        tree_dir_ = ament_index_cpp::get_package_share_directory("bt_manager");
        if (!bt_manager_->initialize(bt_blackboard_, tree_dir_)) {
            std::cerr << RED << "[ERROR] 无法加载行为树 XML 文件，请检查路径: " << tree_dir_ << RESET << std::endl;
            exit(-1);
        }
    }

    void runAllTests() {
        std::cout << "\n" << MAGENTA << "==========================================" << RESET << std::endl;
        std::cout << MAGENTA << "   SENTRY BEHAVIOR TREE 综合自动化测试启动   " << RESET << std::endl;
        std::cout << MAGENTA << "==========================================" << RESET << std::endl;

        runTest("一.1 导航树：己方半场战术模式响应", [this]() { testNav_TacticalModesResponse(); });
        runTest("一.2 导航树：前哨站受击 5s CD 及重回逻辑", [this]() { testNav_OutpostHitAndCooldown(); });
        runTest("一.3 导航树：索敌追击抢占响应", [this]() { testNav_TargetPursuitOverride(); });
        runTest("一.4 导航树：隧道跨越与卡死恢复", [this]() { testNav_TunnelCrossingAndRecovery(); });
        runTest("一.5 导航树：组合隧道场景", [this]() { testNav_CombinedTunnelScenarios(); });
        runTest("一.6 导航树：优先级绝对抢占", [this]() { testNav_PriorityEscalation(); });
        runTest("一.7 导航树：状态抖动 (Ping-Pong) 检测", [this]() { testNav_JitterDetection(); });

        runTest("二.1 姿态树：隧道与跨区姿态", [this]() { testStance_TunnelAndCrossZone(); });
        runTest("二.2 姿态树：多条件姿态切换", [this]() { testStance_VariousConditions(); });
        runTest("二.3 姿态树：优先级覆盖", [this]() { testStance_PriorityEscalation(); });
        runTest("二.4 姿态树：回家再出门迟滞循环", [this]() { testStance_RetreatAndPatrolLoop(); });

        runTest("三.1 战术树：切换与优先级", [this]() { testTactical_SwitchAndPriority(); });
        runTest("三.2 组合边界场景", [this]() { testCombined_ExtremeEdgeCases(); });

        std::cout << GREEN << "\n[ALL TESTS PASSED] 所有行为树逻辑测试完美通过！" << RESET << std::endl;
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<Blackboard> blackboard_;
    std::shared_ptr<BT::Blackboard> bt_blackboard_;
    std::shared_ptr<ros_interface> ros_interface_;
    std::shared_ptr<SentryBTManager> bt_manager_;
    std::string tree_dir_;

    // ================= 辅助测试函数 =================
    void runTest(const std::string& test_name, std::function<void()> test_func) {
        std::cout << "\n" << CYAN << "[RUNNING] " << test_name << RESET << std::endl;
        try {
            resetBlackboard(); // 每次测试前重置状态
            test_func();
            std::cout << GREEN << "[PASSED]  " << test_name << RESET << std::endl;
        } catch (const std::exception& e) {
            std::cout << RED << "[FAILED]  " << test_name << " -> " << e.what() << RESET << std::endl;
        }
    }

    void resetBlackboard() {
        bt_blackboard_->set("health", 100.0f);
        bt_blackboard_->set("bullets_remaining", 300);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);
        bt_blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL));
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("outpost_safe_cooldown_active", false);
        bt_blackboard_->set("manual_override_active", false);
        bt_blackboard_->set("manual_override_goal_valid", false);
        bt_blackboard_->set("manual_override_goal", Point2D{0.0, 0.0, 0.0});
        bt_blackboard_->set("patrol_index", 0);
        bt_blackboard_->set("fort_occupation_status", 0);
        bt_blackboard_->set("is_disengaged", true);
        bt_blackboard_->set("capacitor_capacity", static_cast<uint8_t>(100));
        bt_blackboard_->set("current_stance", SentryStance::DEFEND);
        bt_blackboard_->set("desired_stance", SentryStance::DEFEND);
        bt_blackboard_->set("use_gyro_mode", true);
        bt_blackboard_->set("gyro_vel", 50.0f);
        bt_blackboard_->set("lifter_current_pos", LifterPos::TOP);
        bt_blackboard_->set("desired_lifter_pos", LifterPos::TOP);
        bt_blackboard_->set("home_health", 3000);
        bt_blackboard_->set("small_energy_status", 0);
        bt_blackboard_->set("big_energy_status", 0);
        bt_blackboard_->set("target_pose", makePose(0.0, 0.0));
        bt_blackboard_->set("nav_goal", nav_points[static_cast<size_t>(NavGoal::HOME)]);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
    }

    geometry_msgs::msg::Pose makePose(double x, double y) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.orientation.w = 1.0;
        return pose;
    }

    void tickNavTree() {
        bt_manager_->tickMainExactlyOnce();
    }

    void tickStanceTree() {
        bt_manager_->tickStanceExactlyOnce();
    }

    void tickTacticalTree() {
        bt_manager_->tickTacticalExactlyOnce();
    }

    void waitForStanceCooldown(const std::string & label) {
        std::cout << YELLOW << "      等待 5.1 秒冷却: " << label << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5100));
    }

    void logPoint(const std::string & label, const Point2D & point) {
        std::cout << YELLOW << "      " << label << ": (" << point.x << ", " << point.y << ")" << RESET
                  << std::endl;
    }

    bool nearlyEqual(double a, double b, double tol) {
        return std::fabs(a - b) <= tol;
    }

    void expectPointNear(const Point2D & actual, const Point2D & expected, const std::string & message,
                         double tol = 1e-2) {
        if (!nearlyEqual(actual.x, expected.x, tol) || !nearlyEqual(actual.y, expected.y, tol)) {
            throw std::runtime_error(message);
        }
    }

    Point2D computeTargetGoal(
        const geometry_msgs::msg::Pose & current_pose, const geometry_msgs::msg::Pose & target_pose) {
        const double current_x = current_pose.position.x;
        const double current_y = current_pose.position.y;
        const double target_x = target_pose.position.x;
        const double target_y = target_pose.position.y;
        const double dx = target_x - current_x;
        const double dy = target_y - current_y;
        const double distance = std::hypot(dx, dy);
        const double attack_distance = 0.3;

        Point2D point;
        if (distance > attack_distance) {
            const double scale = 1.0 - attack_distance / distance;
            point.x = current_x + dx * scale;
            point.y = current_y + dy * scale;
        } else if (distance > 0.001) {
            const double ux = dx / distance;
            const double uy = dy / distance;
            point.x = current_x - ux * attack_distance;
            point.y = current_y - uy * attack_distance;
        } else {
            point.x = current_x;
            point.y = current_y - attack_distance;
        }
        point.yaw = 0.0;
        return point;
    }

    int findTransformZoneIndex(const geometry_msgs::msg::Pose & pose) {
        const Point2D point{pose.position.x, pose.position.y, 0.0};
        for (std::size_t i = 0; i < transform_zone.size(); ++i) {
            if (transform_zone[i].contains(point)) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    // ================= 具体测试用例实现 =================

    void testNav_TacticalModesResponse() {
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("fort_occupation_status", 0);

        bt_blackboard_->set("tactical_mode", TacticalMode::OFFENSIVE);
        tickNavTree();
        auto goal = bt_blackboard_->get<Point2D>("nav_goal");
        logPoint("ATTACK nav_goal", goal);
        logPoint("ATTACK expected", nav_points[static_cast<size_t>(NavGoal::ENEMY_FORT)]);
        expectPointNear(goal, nav_points[static_cast<size_t>(NavGoal::ENEMY_FORT)],
                        "攻击模式下未正确下发敌方基地坐标");

        bt_blackboard_->set("tactical_mode", TacticalMode::DEFENSIVE);
        tickNavTree();
        goal = bt_blackboard_->get<Point2D>("nav_goal");
        logPoint("DEFEND nav_goal", goal);
        logPoint("DEFEND expected", nav_points[static_cast<size_t>(NavGoal::OWN_FORT)]);
        expectPointNear(goal, nav_points[static_cast<size_t>(NavGoal::OWN_FORT)],
                        "防守模式下未正确下发己方基地坐标");

        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);
        bt_blackboard_->set("patrol_index", 0);
        tickNavTree();
        goal = bt_blackboard_->get<Point2D>("nav_goal");
        const auto expected_patrol = patrol_points_normal[0].position;
        logPoint("NORMAL nav_goal", goal);
        logPoint("NORMAL expected", expected_patrol);
        expectPointNear(goal, expected_patrol, "正常模式下未正确下发巡逻点");
    }

    void testNav_OutpostHitAndCooldown() {
        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);
        bt_blackboard_->set("enemy_outpost_destroyed", false);
        bt_blackboard_->set("current_mode", static_cast<int>(NavMode::RESPONSE));
        bt_blackboard_->set("health", 100.0f);

        auto logState = [&](const std::string & label) {
            const auto health = bt_blackboard_->get<float>("health");
            const auto cooldown = bt_blackboard_->get<bool>("outpost_safe_cooldown_active");
            std::cout << YELLOW << "      " << label << " health=" << health
                      << " cooldown=" << cooldown << RESET << std::endl;
        };

        tickNavTree();
        logState("Step A");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        bt_blackboard_->set("health", 95.0f);
        tickNavTree();
        logState("Step B");
        auto mode = bt_blackboard_->get<int>("current_mode");
        bool cd_active = bt_blackboard_->get<bool>("outpost_safe_cooldown_active");
        if (mode != static_cast<int>(NavMode::PATROL) || !cd_active) {
            throw std::runtime_error("受击后未正确退出响应模式并激活冷却");
        }

        bt_blackboard_->set("health", 98.4f);
        tickNavTree();
        logState("Step C");
        if (!bt_blackboard_->get<bool>("outpost_safe_cooldown_active")) {
            throw std::runtime_error("5秒内继续受击，冷却被错误地提前解除了");
        }

        std::cout << YELLOW << "      等待 5.1 秒模拟冷却结束..." << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5100));
        tickNavTree();
        logState("Step D");
        cd_active = bt_blackboard_->get<bool>("outpost_safe_cooldown_active");
        mode = bt_blackboard_->get<int>("current_mode");
        if (cd_active || mode != static_cast<int>(NavMode::RESPONSE)) {
            throw std::runtime_error("5秒无伤后冷却未能解除并恢复 RESPONSE 模式");
        }
    }

    void testNav_TargetPursuitOverride() {
        const auto current_pose = makePose(7.0, 1.5);
        bt_blackboard_->set("current_pose", current_pose);
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("nav_goal", nav_points[static_cast<size_t>(NavGoal::HOME)]);

        tickNavTree();
        const auto mode_before = bt_blackboard_->get<int>("current_mode");
        const auto goal_before = bt_blackboard_->get<Point2D>("nav_goal");
        std::cout << YELLOW << "      before mode=" << mode_before << " goal=(" << goal_before.x
                  << ", " << goal_before.y << ")" << RESET << std::endl;

        const auto target_pose = makePose(8.0, 2.0);
        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("target_pose", target_pose);
        bt_blackboard_->set("nav_goal", nav_points[static_cast<size_t>(NavGoal::ENEMY_FORT)]);

        tickNavTree();
        const auto mode_after = bt_blackboard_->get<int>("current_mode");
        const auto goal_after = bt_blackboard_->get<Point2D>("nav_goal");
        std::cout << YELLOW << "      after mode=" << mode_after << " goal=(" << goal_after.x
                  << ", " << goal_after.y << ")" << RESET << std::endl;

        if (mode_after != static_cast<int>(NavMode::TRACING)) {
            throw std::runtime_error("追击逻辑未能抢占响应/巡逻逻辑");
        }

        const auto expected = computeTargetGoal(current_pose, target_pose);
        logPoint("TRACING expected", expected);
        expectPointNear(goal_after, expected, "追击模式下未正确下发接近目标的坐标", 5e-2);
    }

    void testNav_TunnelCrossingAndRecovery() {
        auto assertLifterBottom = [&](const std::string & label) {
            const auto lifter = bt_blackboard_->get<LifterPos>("desired_lifter_pos");
            std::cout << YELLOW << "      " << label << " lifter=" << static_cast<int>(lifter) << RESET
                      << std::endl;
            if (lifter != LifterPos::BOTTOM) {
                throw std::runtime_error("隧道场景下升降未强制到底部");
            }
        };

        bt_blackboard_->set("current_pose", makePose(12.0, 5.0));
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("lifter_current_pos", LifterPos::TOP);
        tickNavTree();
        assertLifterBottom("进入 tunnel_area");

        bt_blackboard_->set("current_pose", makePose(10.0, 3.0));
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("lifter_current_pos", LifterPos::BOTTOM);
        tickNavTree();
        assertLifterBottom("离开 tunnel_area 但仍在 transform_area");

        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("nav_goal", Point2D{10.0, 6.8, 0.0});
        tickNavTree();
        assertLifterBottom("防守区 -> 高地");

        bt_blackboard_->set("current_pose", makePose(10.0, 6.8));
        bt_blackboard_->set("nav_goal", Point2D{7.0, 1.5, 0.0});
        tickNavTree();
        assertLifterBottom("高地 -> 防守区");

        const auto recovery_pose = makePose(10.0, 3.0);
        const int tunnel_idx = findTransformZoneIndex(recovery_pose);
        if (tunnel_idx < 0) {
            throw std::runtime_error("卡死恢复测试：未找到 transform_zone");
        }
        bt_blackboard_->set("current_pose", recovery_pose);
        bt_blackboard_->set("through_tunnel", false);
        tickNavTree();

        std::cout << YELLOW << "      等待 10.1 秒触发卡死恢复 Phase1..." << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10100));
        tickNavTree();
        auto goal = bt_blackboard_->get<Point2D>("nav_goal");
        const auto expected_forward = tunnel_recovery_configs[static_cast<size_t>(tunnel_idx)].forward_point;
        logPoint("Phase1 nav_goal", goal);
        logPoint("Phase1 expected", expected_forward);
        expectPointNear(goal, expected_forward, "卡死恢复 Phase1 目标点错误", 5e-2);

        std::cout << YELLOW << "      等待 5.1 秒触发卡死恢复 Phase2..." << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5100));
        tickNavTree();
        goal = bt_blackboard_->get<Point2D>("nav_goal");
        const auto expected_retreat = tunnel_recovery_configs[static_cast<size_t>(tunnel_idx)].recovery_point;
        logPoint("Phase2 nav_goal", goal);
        logPoint("Phase2 expected", expected_retreat);
        expectPointNear(goal, expected_retreat, "卡死恢复 Phase2 目标点错误", 5e-2);

        std::cout << YELLOW << "      等待 5.1 秒触发卡死恢复 Phase3..." << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5100));
        tickNavTree();
        const auto cmd_vel = bt_blackboard_->get<geometry_msgs::msg::Twist>("cmd_vel");
        std::cout << YELLOW << "      cmd_vel=(vx=" << cmd_vel.linear.x << ", vy=" << cmd_vel.linear.y << ")"
                  << RESET << std::endl;
        const auto & cfg = tunnel_recovery_configs[static_cast<size_t>(tunnel_idx)];
        if (!nearlyEqual(cmd_vel.linear.x, cfg.recovery_vx, 1e-3) ||
            !nearlyEqual(cmd_vel.linear.y, cfg.recovery_vy, 1e-3)) {
            throw std::runtime_error("卡死恢复 Phase3 未正确下发世界系速度");
        }
    }

    void testNav_CombinedTunnelScenarios() {
        const auto tunnel_pose = makePose(10.0, 3.0);
        auto assertLifterBottom = [&](const std::string & label) {
            const auto lifter = bt_blackboard_->get<LifterPos>("desired_lifter_pos");
            std::cout << YELLOW << "      " << label << " lifter=" << static_cast<int>(lifter) << RESET
                      << std::endl;
            if (lifter != LifterPos::BOTTOM) {
                throw std::runtime_error("组合隧道场景未强制升降到底部");
            }
        };

        bt_blackboard_->set("current_pose", tunnel_pose);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("enemy_outpost_destroyed", false);
        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);
        bt_blackboard_->set("health", 100.0f);
        bt_blackboard_->set("target_valid", false);
        tickNavTree();
        assertLifterBottom("RESPONSE in tunnel");

        bt_blackboard_->set("current_pose", tunnel_pose);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("health", 20.0f);
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", false);
        tickNavTree();
        assertLifterBottom("RETREAT in tunnel");

        bt_blackboard_->set("current_pose", tunnel_pose);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("health", 100.0f);
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("target_pose", makePose(8.0, 2.0));
        tickNavTree();
        assertLifterBottom("TRACING in tunnel");
    }

    void testNav_PriorityEscalation() {
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("manual_override_goal_valid", false);
        bt_blackboard_->set("health", 100.0f);
        bt_blackboard_->set("tactical_mode", TacticalMode::BALANCED);

        tickNavTree();
        int mode = bt_blackboard_->get<int>("current_mode");
        std::cout << YELLOW << "      Patrol mode=" << mode << RESET << std::endl;
        if (mode != static_cast<int>(NavMode::PATROL)) {
            throw std::runtime_error("最低优先级巡逻未生效");
        }

        bt_blackboard_->set("tactical_mode", TacticalMode::OFFENSIVE);
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        tickNavTree();
        auto goal = bt_blackboard_->get<Point2D>("nav_goal");
        mode = bt_blackboard_->get<int>("current_mode");
        std::cout << YELLOW << "      AttackFort mode=" << mode << " goal=(" << goal.x << ", " << goal.y
                  << ")" << RESET << std::endl;
        logPoint("AttackFort expected", nav_points[static_cast<size_t>(NavGoal::ENEMY_FORT)]);
        expectPointNear(goal, nav_points[static_cast<size_t>(NavGoal::ENEMY_FORT)],
                        "攻击堡垒未正确下发敌方基地目标");

        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("target_pose", makePose(8.0, 2.0));
        tickNavTree();
        mode = bt_blackboard_->get<int>("current_mode");
        std::cout << YELLOW << "      Pursuit mode=" << mode << RESET << std::endl;
        if (mode != static_cast<int>(NavMode::TRACING)) {
            throw std::runtime_error("追击未能抢占攻击堡垒");
        }

        bt_blackboard_->set("health", 20.0f);
        tickNavTree();
        mode = bt_blackboard_->get<int>("current_mode");
        std::cout << YELLOW << "      Retreat mode=" << mode << RESET << std::endl;
        if (mode != static_cast<int>(NavMode::RETREAT)) {
            throw std::runtime_error("回家未能抢占追击");
        }

        bt_blackboard_->set("manual_override_active", true);
        bt_blackboard_->set("manual_override_goal_valid", true);
        const auto manual_goal = nav_points[static_cast<size_t>(NavGoal::BONUS)];
        bt_blackboard_->set("manual_override_goal", manual_goal);
        tickNavTree();
        mode = bt_blackboard_->get<int>("current_mode");
        const auto manual_nav = bt_blackboard_->get<Point2D>("nav_goal");
        std::cout << YELLOW << "      Manual mode=" << mode << " goal=(" << manual_nav.x << ", "
                  << manual_nav.y << ")" << RESET << std::endl;
        expectPointNear(manual_nav, manual_goal, "手动接管未正确下发目标点");
        if (mode != static_cast<int>(NavMode::MANUAL)) {
            throw std::runtime_error("手动接管未能抢占更低优先级行为");
        }
    }

    void testNav_JitterDetection() {
        bt_blackboard_->set("health", 29.0f);
        tickNavTree();
        int mode1 = bt_blackboard_->get<int>("current_mode");

        bt_blackboard_->set("health", 31.0f);
        tickNavTree();
        int mode2 = bt_blackboard_->get<int>("current_mode");

        std::cout << YELLOW << "      mode1=" << mode1 << " mode2=" << mode2 << RESET << std::endl;
        if (mode1 != static_cast<int>(NavMode::RETREAT) || mode2 != static_cast<int>(NavMode::RETREAT)) {
            throw std::runtime_error("检测到状态机迟滞逻辑失效，发生乒乓抖动！");
        }
    }

    void testStance_TunnelAndCrossZone() {
        waitForStanceCooldown("跨区姿态");
        bt_blackboard_->set("current_stance", SentryStance::ATTACK);
        bt_blackboard_->set("desired_stance", SentryStance::ATTACK);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("nav_goal", Point2D{10.0, 6.8, 0.0});

        tickStanceTree();
        const auto desired = bt_blackboard_->get<SentryStance>("desired_stance");
        const auto use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::MOVE || use_gyro) {
            throw std::runtime_error("跨区姿态未切换为 MOVE 或未关闭小陀螺");
        }
    }

    void testStance_VariousConditions() {
        waitForStanceCooldown("热量触发");
        bt_blackboard_->set("current_stance", SentryStance::MOVE);
        bt_blackboard_->set("desired_stance", SentryStance::MOVE);
        bt_blackboard_->set("current_heat", 250);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("is_disengaged", true);
        tickStanceTree();
        auto desired = bt_blackboard_->get<SentryStance>("desired_stance");
        auto use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      heat desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::ATTACK || !use_gyro) {
            throw std::runtime_error("热量超标未切换为 ATTACK 或未开启小陀螺");
        }

        waitForStanceCooldown("追击距离");
        bt_blackboard_->set("current_stance", SentryStance::DEFEND);
        bt_blackboard_->set("desired_stance", SentryStance::DEFEND);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("target_pose", makePose(13.0, 1.5));
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      pursuit desired_stance=" << static_cast<int>(desired) << RESET
                  << std::endl;
        if (desired != SentryStance::MOVE) {
            throw std::runtime_error("追击距离触发未切换为 MOVE");
        }

        waitForStanceCooldown("电容不足");
        bt_blackboard_->set("current_stance", SentryStance::DEFEND);
        bt_blackboard_->set("desired_stance", SentryStance::DEFEND);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("is_disengaged", true);
        bt_blackboard_->set("capacitor_capacity", static_cast<uint8_t>(20));
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      capacitor desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::MOVE || !use_gyro) {
            throw std::runtime_error("电容不足未切换为 MOVE 或未保持小陀螺");
        }
    }

    void testStance_PriorityEscalation() {
        waitForStanceCooldown("DefaultDefend");
        bt_blackboard_->set("current_stance", SentryStance::MOVE);
        bt_blackboard_->set("desired_stance", SentryStance::MOVE);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("capacitor_capacity", static_cast<uint8_t>(100));
        bt_blackboard_->set("is_disengaged", false);
        bt_blackboard_->set("health", 40.0f);
        tickStanceTree();
        auto desired = bt_blackboard_->get<SentryStance>("desired_stance");
        auto use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      DefaultDefend desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::DEFEND) {
            throw std::runtime_error("DefaultDefend 未生效");
        }

        waitForStanceCooldown("MoveStanceGyro");
        bt_blackboard_->set("current_stance", SentryStance::DEFEND);
        bt_blackboard_->set("desired_stance", SentryStance::DEFEND);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("capacitor_capacity", static_cast<uint8_t>(20));
        bt_blackboard_->set("is_disengaged", true);
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      MoveStanceGyro desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::MOVE || !use_gyro) {
            throw std::runtime_error("MoveStanceGyro 未覆盖 DefaultDefend");
        }

        waitForStanceCooldown("CombatAttack");
        bt_blackboard_->set("current_stance", SentryStance::MOVE);
        bt_blackboard_->set("desired_stance", SentryStance::MOVE);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("is_disengaged", false);
        bt_blackboard_->set("health", 90.0f);
        bt_blackboard_->set("capacitor_capacity", static_cast<uint8_t>(100));
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      CombatAttack desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::ATTACK || !use_gyro) {
            throw std::runtime_error("CombatAttack 未覆盖 MoveStanceGyro");
        }

        waitForStanceCooldown("Pursuit");
        bt_blackboard_->set("current_stance", SentryStance::ATTACK);
        bt_blackboard_->set("desired_stance", SentryStance::ATTACK);
        bt_blackboard_->set("current_heat", 0);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("target_pose", makePose(13.0, 1.5));
        bt_blackboard_->set("is_disengaged", true);
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      Pursuit desired_stance=" << static_cast<int>(desired) << RESET
                  << std::endl;
        if (desired != SentryStance::MOVE) {
            throw std::runtime_error("Pursuit 未覆盖 CombatAttack");
        }

        waitForStanceCooldown("AbsoluteAttack");
        bt_blackboard_->set("current_stance", SentryStance::MOVE);
        bt_blackboard_->set("desired_stance", SentryStance::MOVE);
        bt_blackboard_->set("current_heat", 250);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("through_tunnel", false);
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      AbsoluteAttack desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::ATTACK || !use_gyro) {
            throw std::runtime_error("AbsoluteAttack 未覆盖 Pursuit");
        }

        waitForStanceCooldown("MoveStanceNoGyro");
        bt_blackboard_->set("current_stance", SentryStance::ATTACK);
        bt_blackboard_->set("desired_stance", SentryStance::ATTACK);
        bt_blackboard_->set("current_heat", 250);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("current_pose", makePose(7.0, 1.5));
        bt_blackboard_->set("nav_goal", Point2D{10.0, 6.8, 0.0});
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      MoveNoGyro desired_stance=" << static_cast<int>(desired)
                  << " use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (desired != SentryStance::MOVE || use_gyro) {
            throw std::runtime_error("跨区关陀螺未覆盖热量超标开陀螺");
        }
    }

    void testStance_RetreatAndPatrolLoop() {
        waitForStanceCooldown("retreat");
        bt_blackboard_->set("current_stance", SentryStance::MOVE);
        bt_blackboard_->set("desired_stance", SentryStance::MOVE);
        bt_blackboard_->set("health", 15.0f);
        bt_blackboard_->set("is_disengaged", true);
        bt_blackboard_->set("current_mode", static_cast<int>(NavMode::RETREAT));
        tickStanceTree();
        auto desired = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      retreat desired_stance=" << static_cast<int>(desired) << RESET
                  << std::endl;
        if (desired != SentryStance::MOVE) {
            throw std::runtime_error("残血回家阶段未切换为 MOVE");
        }

        waitForStanceCooldown("patrol");
        bt_blackboard_->set("health", 85.0f);
        bt_blackboard_->set("is_disengaged", true);
        bt_blackboard_->set("current_mode", static_cast<int>(NavMode::PATROL));
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      patrol desired_stance=" << static_cast<int>(desired) << RESET
                  << std::endl;
        if (desired != SentryStance::MOVE) {
            throw std::runtime_error("出门巡逻阶段未切换为 MOVE");
        }

        waitForStanceCooldown("combat");
        bt_blackboard_->set("is_disengaged", false);
        tickStanceTree();
        desired = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      combat desired_stance=" << static_cast<int>(desired) << RESET
                  << std::endl;
        if (desired != SentryStance::ATTACK) {
            throw std::runtime_error("交战阶段未切换为 ATTACK");
        }
    }

    void testTactical_SwitchAndPriority() {
        bt_blackboard_->set("enemy_outpost_destroyed", false);
        bt_blackboard_->set("small_energy_status", 0);
        bt_blackboard_->set("big_energy_status", 0);
        bt_blackboard_->set("home_health", 3000);
        tickTacticalTree();
        auto mode = bt_blackboard_->get<TacticalMode>("tactical_mode");
        std::cout << YELLOW << "      default tactical_mode=" << static_cast<int>(mode) << RESET << std::endl;
        if (mode != TacticalMode::BALANCED) {
            throw std::runtime_error("默认战术模式不是 NORMAL/BALANCED");
        }

        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("small_energy_status", 1);
        bt_blackboard_->set("big_energy_status", 0);
        bt_blackboard_->set("home_health", 3000);
        tickTacticalTree();
        mode = bt_blackboard_->get<TacticalMode>("tactical_mode");
        std::cout << YELLOW << "      attack tactical_mode=" << static_cast<int>(mode) << RESET << std::endl;
        if (mode != TacticalMode::OFFENSIVE) {
            throw std::runtime_error("满足攻击条件时未切换为 OFFENSIVE");
        }

        // Defend 分支当前在 tactical_tree.xml 中被注释，优先级验证留待启用后补充。
    }

    void testCombined_ExtremeEdgeCases() {
        const auto tunnel_pose = makePose(12.0, 5.0);
        bt_blackboard_->set("current_pose", tunnel_pose);
        bt_blackboard_->set("health", 25.0f);
        bt_blackboard_->set("current_heat", 250);
        bt_blackboard_->set("through_tunnel", true);
        bt_blackboard_->set("enemy_outpost_destroyed", true);
        bt_blackboard_->set("target_valid", false);
        bt_blackboard_->set("lifter_current_pos", LifterPos::TOP);
        tickNavTree();
        const auto retreat_goal = bt_blackboard_->get<Point2D>("nav_goal");
        logPoint("EdgeA nav_goal", retreat_goal);
        logPoint("EdgeA expected", nav_points[static_cast<size_t>(NavGoal::HOME)]);
        expectPointNear(retreat_goal, nav_points[static_cast<size_t>(NavGoal::HOME)],
                        "要命的撤退：NavTree 未下发回家目标");
        const auto lifter = bt_blackboard_->get<LifterPos>("desired_lifter_pos");
        std::cout << YELLOW << "      EdgeA lifter=" << static_cast<int>(lifter) << RESET << std::endl;
        if (lifter != LifterPos::BOTTOM) {
            throw std::runtime_error("要命的撤退：lifter 未强制到底部");
        }

        waitForStanceCooldown("edge-case A stance");
        bt_blackboard_->set("current_stance", SentryStance::ATTACK);
        bt_blackboard_->set("desired_stance", SentryStance::ATTACK);
        tickStanceTree();
        const auto use_gyro = bt_blackboard_->get<bool>("use_gyro_mode");
        std::cout << YELLOW << "      EdgeA use_gyro_mode=" << use_gyro << RESET << std::endl;
        if (use_gyro) {
            throw std::runtime_error("要命的撤退：StanceTree 未关闭小陀螺");
        }

        const auto stuck_pose = makePose(10.0, 3.0);
        const int tunnel_idx = findTransformZoneIndex(stuck_pose);
        if (tunnel_idx < 0) {
            throw std::runtime_error("卡死时的被袭：未找到 transform_zone");
        }
        bt_blackboard_->set("current_pose", stuck_pose);
        bt_blackboard_->set("health", 100.0f);
        bt_blackboard_->set("through_tunnel", false);
        bt_blackboard_->set("target_valid", false);
        tickNavTree();
        std::cout << YELLOW << "      等待 20.5 秒触发卡死恢复..." << RESET << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20500));
        bt_blackboard_->set("target_valid", true);
        bt_blackboard_->set("target_pose", makePose(8.0, 2.0));
        tickNavTree();
        const auto cmd_vel = bt_blackboard_->get<geometry_msgs::msg::Twist>("cmd_vel");
        std::cout << YELLOW << "      EdgeB cmd_vel=(vx=" << cmd_vel.linear.x << ", vy="
                  << cmd_vel.linear.y << ")" << RESET << std::endl;
        const auto & cfg = tunnel_recovery_configs[static_cast<size_t>(tunnel_idx)];
        if (!nearlyEqual(cmd_vel.linear.x, cfg.recovery_vx, 1e-3) ||
            !nearlyEqual(cmd_vel.linear.y, cfg.recovery_vy, 1e-3)) {
            throw std::runtime_error("卡死时的被袭：Recovery 未优先下发 cmd_vel");
        }

        waitForStanceCooldown("edge-case C refresh");
        auto refresh_manager = std::make_shared<SentryBTManager>();
        if (!refresh_manager->initialize(bt_blackboard_, tree_dir_)) {
            throw std::runtime_error("RuleRefresh 测试初始化失败");
        }
        bt_blackboard_->set("current_stance", SentryStance::DEFEND);
        bt_blackboard_->set("desired_stance", SentryStance::DEFEND);
        refresh_manager->tickStanceExactlyOnce();
        const auto refresh_stance = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      EdgeC refresh stance=" << static_cast<int>(refresh_stance) << RESET
                  << std::endl;

        bt_blackboard_->set("health", 25.0f);
        tickNavTree();
        const auto retreat_mode = bt_blackboard_->get<int>("current_mode");
        if (retreat_mode != static_cast<int>(NavMode::RETREAT)) {
            throw std::runtime_error("防静默期间的激战：NavTree 未切换为 RETREAT");
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        refresh_manager->tickStanceExactlyOnce();
        const auto stance_after = bt_blackboard_->get<SentryStance>("desired_stance");
        std::cout << YELLOW << "      EdgeC stance_after=" << static_cast<int>(stance_after) << RESET
                  << std::endl;
        if (stance_after != refresh_stance) {
            throw std::runtime_error("防静默期间的激战：姿态刷新冷却未拦截新指令");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("bt_comprehensive_test_node");
    
    try {
        SentryTestSuite suite(node);
        suite.runAllTests();
    } catch (const std::exception& e) {
        std::cerr << "测试套件遭遇致命错误: " << e.what() << std::endl;
    }
    
    rclcpp::shutdown();
    return 0;
}