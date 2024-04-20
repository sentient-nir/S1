#ifndef __SENTIENT_PLANNER_H__
#define __SENTIENT_PLANNER_H__

#include <signal.h>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
// #include <moveit_visual_tools/moveit_visual_tools.h>

#include <std_msgs/msg/bool.hpp>
#include <xarm_msgs/srv/plan_pose.hpp>
#include <xarm_msgs/srv/plan_joint.hpp>
#include <xarm_msgs/srv/plan_exec.hpp>
#include <xarm_msgs/srv/plan_single_straight.hpp>


namespace sentient_planner
{
    class SentientPlanner
    {
    public:
        SentientPlanner(const rclcpp::Node::SharedPtr& node, const std::string& group_name);
        SentientPlanner(const std::string& group_name);
        ~SentientPlanner() {};

        bool planJointTarget(const std::vector<double>& joint_target);
        bool planPoseTarget(const geometry_msgs::msg::Pose& pose_target);
        bool planPoseTargets(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);
        bool planCartesianPath(const std::vector<geometry_msgs::msg::Pose>& pose_target_vector);

        bool executePath(bool wait = true);
    private:
        void init(const std::string& group_name);

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        moveit::planning_interface::MoveGroupInterface::Plan sentient_plan_;
        moveit_msgs::msg::RobotTrajectory trajectory_;
        bool is_trajectory_;
    };
}

#endif // __SENTIENT_PLANNER_H__
