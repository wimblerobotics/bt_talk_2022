#include "moveToPose.hpp"

#include <behaviortree_cpp_v3/behavior_tree.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

// #include "moveToPoseActionClient.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#undef FUTURE_WAIT_BLOCK

using std::placeholders::_1;
using std::placeholders::_2;

MoveToPose::MoveToPose(const std::string& name,
                       const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config), aborted_(false) {
  node_ = rclcpp::Node::make_shared("move_to_pose_client_ptr_");
  client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
      node_->get_node_base_interface(), node_->get_node_graph_interface(),
      node_->get_node_logging_interface(),
      node_->get_node_waitables_interface(),
      /*  node_,*/ "navigate_to_pose");
  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
}

BT::NodeStatus MoveToPose::tick() {
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  transform_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Take the goal from the InputPort of the Node
  geometry_msgs::msg::Pose goal;
  if (!getInput<geometry_msgs::msg::Pose>("goal", goal)) {
    throw BT::RuntimeError("[MoveToPose::tick] missing required input [goal]");
  }

  auto timeout_ms_ = std::chrono::milliseconds(2);
  while (!tf_buffer_->canTransform("map", "base_link", tf2::TimePointZero) &&
         rclcpp::ok()) {
    std::this_thread::sleep_for(timeout_ms_);
  }

  geometry_msgs::msg::TransformStamped transformStamped =
      tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

  geometry_msgs::msg::PoseStamped psIn;
  geometry_msgs::msg::PoseStamped psOut;
  psIn.header.frame_id = "base_link";
  psIn.header.stamp = node_->get_clock()->now();
  psIn.pose.position = goal.position;
  psIn.pose.orientation.w = 1.0;
  psIn.pose.orientation.x = 0.0;
  psIn.pose.orientation.y = 0.0;
  psIn.pose.orientation.z = 0.0;

  geometry_msgs::msg::PointStamped transformed_pt;
  tf2::doTransform(psIn, psOut, transformStamped);

  float goalx = psOut.pose.position.x;
  float goaly = psOut.pose.position.y;
  RCLCPP_INFO(node_->get_logger(),
              "[MoveToPose::tick] given xlx: %7.3f, xly: %7.3f, transform goal "
              "x: %7.3f, y: %7.3f to map x: "
              "%7.3f, y: %7.3f",
              transformStamped.transform.translation.x,
              transformStamped.transform.translation.y, goal.position.x,
              goal.position.y, goalx, goaly);
  nav2_msgs::action::NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = node_->get_clock()->now();
  goal_msg.pose.pose.position.x = goalx;
  goal_msg.pose.pose.position.y = goaly;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.w = 1.0;
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = 0.0;
  goal_msg.behavior_tree = "";

  auto goal_handle_future = client_ptr_->async_send_goal(goal_msg);
  RCLCPP_INFO(node_->get_logger(), "Sending goal");
  if (rclcpp::spin_until_future_complete(node_, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "send goal call failed : (");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return BT::NodeStatus::FAILURE;
  }

  auto result_future = client_ptr_->async_get_result(goal_handle);

  RCLCPP_INFO(node_->get_logger(), "Waiting for result");
  if (rclcpp::spin_until_future_complete(node_, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "get result call failed :  (");
    return BT::NodeStatus::FAILURE;
  }

  rclcpp_action::ClientGoalHandle<
      nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result =
      result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
      return BT::NodeStatus::FAILURE;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
      return BT::NodeStatus::FAILURE;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
      return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "result received");

  return BT::NodeStatus::SUCCESS;
}

BT::PortsList MoveToPose::providedPorts() {
  // Optionally, a port can have a human readable description
  return {BT::InputPort<geometry_msgs::msg::Pose>("goal")};
}

void MoveToPose::halt() {
  std::cout << "[MoveToPose] requesting nav abort" << std::endl;
  aborted_ = true;
}