#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

class MoveToPoseActionClient : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  explicit MoveToPoseActionClient(const rclcpp::NodeOptions& options)
      : Node("move_to_pose_action_client", options) {
    this->client_ptr_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void send_goal() {
    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    nav2_msgs::action::NavigateToPose::Goal goal_msg;
    goal_msg.pose.header.frame_id = "map";  //#####"base_link";
    goal_msg.pose.header.stamp = get_clock()->now();
    goal_msg.pose.pose.position.x = 3;  //#####goal_in.position.x;
    goal_msg.pose.pose.position.y = 0;  //#####goal_in.position.y;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.behavior_tree = "";

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MoveToPoseActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MoveToPoseActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MoveToPoseActionClient::result_callback, this, _1);
    auto x = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    std::cout << "send goal sent, await future complete " << std::endl;
    auto y = rclcpp::spin_until_future_complete(this->get_node_base_interface(), x);
    std::cout << "send goal future complete " << y << std::endl;
  }

 private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;

  void goal_response_callback(
      std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    std::stringstream ss;
    ss << "feedback_callback() called,, distance_remaining: " <<  feedback->distance_remaining;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class FibonacciActionClient

RCLCPP_COMPONENTS_REGISTER_NODE(MoveToPoseActionClient)