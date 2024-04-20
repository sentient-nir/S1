#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include "sentient_planner/sentient_planner.h"
#include <std_msgs/msg/bool.hpp>
#include <sentient_interfaces/srv/plan_pose.hpp>
#include <sentient_interfaces/srv/plan_joint.hpp>
#include <sentient_interfaces/srv/plan_exec.hpp>
#include <sentient_interfaces/srv/plan_single_straight.hpp>

#define BIND_CLS_CB(func) std::bind(func, this, std::placeholders::_1, std::placeholders::_2)

class SentientPlannerRunner
{
public:
    SentientPlannerRunner(rclcpp::Node::SharedPtr& node);
    ~SentientPlannerRunner() {};

private:
    bool do_pose_plan(const std::shared_ptr<sentient_interfaces::srv::PlanPose::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanPose::Response> res);
    bool do_joint_plan(const std::shared_ptr<sentient_interfaces::srv::PlanJoint::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanJoint::Response> res);
    bool exec_plan_cb(const std::shared_ptr<sentient_interfaces::srv::PlanExec::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanExec::Response> res);
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<sentient_planner::sentientPlanner> sentient_planner_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<sentient_interfaces::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<sentient_interfaces::srv::PlanPose>::SharedPtr pose_plan_server_;
    rclcpp::Service<sentient_interfaces::srv::PlanJoint>::SharedPtr joint_plan_server_;
    rclcpp::Service<sentient_interfaces::srv::PlanSingleStraight>::SharedPtr single_straight_plan_server_;
};

SentientPlannerRunner::SentientPlannerRunner(rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    int dof;
    node_->get_parameter_or("dof", dof, 7);
    std::string robot_type;
    node_->get_parameter_or("robot_type", robot_type, std::string("sentient"));
    std::string group_name = robot_type;
    if (robot_type == "sentient" || robot_type == "lite")
        group_name = robot_type + std::to_string(dof);
    std::string prefix;
    node->get_parameter_or("prefix", prefix, std::string(""));
    if (prefix != "") {
        group_name = prefix + group_name;
    }

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    sentient_planner_ = std::make_shared<sentient_planner::sentientPlanner>(group_name);

    exec_plan_server_ = node_->create_service<sentient_interfaces::srv::PlanExec>("sentient_exec_plan", BIND_CLS_CB(&SentientPlannerRunner::exec_plan_cb));
    pose_plan_server_ = node_->create_service<sentient_interfaces::srv::PlanPose>("sentient_pose_plan", BIND_CLS_CB(&SentientPlannerRunner::do_pose_plan));
    joint_plan_server_ = node_->create_service<sentient_interfaces::srv::PlanJoint>("sentient_joint_plan", BIND_CLS_CB(&SentientPlannerRunner::do_joint_plan));
}

bool SentientPlannerRunner::do_pose_plan(const std::shared_ptr<sentient_interfaces::srv::PlanPose::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanPose::Response> res)
{
    bool success = sentient_planner_->planPoseTarget(req->target);
    res->success = success;
    return success;
}

bool SentientPlannerRunner::do_joint_plan(const std::shared_ptr<sentient_interfaces::srv::PlanJoint::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanJoint::Response> res)
{
    bool success = sentient_planner_->planJointTarget(req->target);
    res->success = success;
    return success;
}

bool SentientPlannerRunner::exec_plan_cb(const std::shared_ptr<sentient_interfaces::srv::PlanExec::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanExec::Response> res)
{
    bool success = sentient_planner_->executePath(req->wait);
    res->success = success;
    return success;
}

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[sentient_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sentient_planner_node", node_options);
    RCLCPP_INFO(node->get_logger(), "sentient_planner_node start");
    signal(SIGINT, exit_sig_handler);

    SentientPlannerRunner sentient_planner_runner(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "sentient_planner_node over");
    return 0;
}