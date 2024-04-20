#include "sentient_planner/sentient_planner.h"
#include <thread>
#include <mutex>

namespace sentient_planner
{
class DualArmCoordinator {
public:
    DualArmCoordinator(const rclcpp::Node::SharedPtr& node,
                       const std::string& left_group, const std::string& right_group)
        : left_planner_(node, left_group), right_planner_(node, right_group),
          left_thread_(), right_thread_(), sync_mutex_() {}

    bool planAndExecuteJoints(const std::vector<double>& left_joint_targets, const std::vector<double>& right_joint_targets) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        bool left_ready = false, right_ready = false;

        left_thread_ = std::thread([&]() {
            left_ready = left_planner_.planJointTarget(left_joint_targets);
        });

        right_thread_ = std::thread([&]() {
            right_ready = right_planner_.planJointTarget(right_joint_targets);
        });

        left_thread_.join();
        right_thread_.join();

        if (left_ready && right_ready) {
            return left_planner_.executePath(true) && right_planner_.executePath(true);
        }
        return false;
    }

private:
    SentientPlanner left_planner_;
    SentientPlanner right_planner_;
    std::thread left_thread_;
    std::thread right_thread_;
    std::mutex sync_mutex_;
};
}