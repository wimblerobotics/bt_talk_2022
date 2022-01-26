#include "closePointSubscriber.hpp"

#include <cstdio>
#include <laser_lines/msg/closest_point_to_line_segment.hpp>
#include <laser_lines/msg/closest_point_to_line_segment_list.hpp>

#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

ClosePointSubscriber::ClosePointSubscriber() : Node("close_point_subscriber") {
  rclcpp::QoS bestEffortQos(10);
  bestEffortQos.keep_last(10);
  bestEffortQos.best_effort();
  bestEffortQos.durability_volatile();

  closest_line_subscriber_ =
      create_subscription<laser_lines::msg::ClosestPointToLineSegmentList>(
          "/closest_point_to_line_segments", bestEffortQos,
          std::bind(&ClosePointSubscriber::closestPointCallback, this, _1));

  RCLCPP_INFO(this->get_logger(), "ClosePointSubscriber constructed");
}

void ClosePointSubscriber::closestPointCallback(
    const laser_lines::msg::ClosestPointToLineSegmentList::SharedPtr msg) {
//   RCLCPP_INFO(this->get_logger(), "Message received");
  line_segments_.clear();
  for (size_t i = 0; i < msg->closest_point_to_line_segments.size(); i++) {
    line_segments_.push_back(msg->closest_point_to_line_segments[i]);
  }

//   RCLCPP_INFO(this->get_logger(), "Message received, pushed %ld segments",
//               line_segments_.size());
}
