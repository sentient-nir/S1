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
    bool do_joint_plan(const std::shared_ptr<sentient_interfaces::srv::PlanJoint::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanJoint::Response> res);
    bool exec_plan_cb(const std::shared_ptr<sentient_interfaces::srv::PlanExec::Request> req, std::shared_ptr<sentient_interfaces::srv::PlanExec::Response> res);
    
private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<sentient_planner::sentientPlanner> sentient_planner_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr exec_plan_sub_;

    rclcpp::Service<sentient_interfaces::srv::PlanExec>::SharedPtr exec_plan_server_;
    rclcpp::Service<sentient_interfaces::srv::PlanJoint>::SharedPtr joint_plan_server_;
};

SentientPlannerRunner::SentientPlannerRunner(rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    std::string group_name;   
    node_->get_parameter_or("PLANNING_GROUP", group_name, std::string("sentient_gripper"));

    RCLCPP_INFO(node_->get_logger(), "namespace: %s, group_name: %s", node->get_namespace(), group_name.c_str());

    sentient_planner_ = std::make_shared<sentient_planner::sentientPlanner>(group_name);

    exec_plan_server_ = node_->create_service<sentient_interfaces::srv::PlanExec>("sentient_gripper_exec_plan", BIND_CLS_CB(&SentientPlannerRunner::exec_plan_cb));
    joint_plan_server_ = node_->create_service<sentient_interfaces::srv::PlanJoint>("sentient_gripper_joint_plan", BIND_CLS_CB(&SentientPlannerRunner::do_joint_plan));
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
    fprintf(stderr, "[sentient_gripper_planner_node] Ctrl-C caught, exit process...\n");
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sentient_gripper_planner_node", node_options);
    RCLCPP_INFO(node->get_logger(), "sentient_gripper_planner_node start");
    signal(SIGINT, exit_sig_handler);

    SentientPlannerRunner sentient_planner_runner(node);
    
    rclcpp::spin(node);
    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "sentient_gripper_planner_node over");
    return 0;
}