#pragma once

#include <cstdio>
#include <laser_lines/msg/closest_point_to_line_segment.hpp>
#include <laser_lines/msg/closest_point_to_line_segment_list.hpp>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std;

class ClosePointSubscriber : public rclcpp::Node {
 public:
  ClosePointSubscriber(ClosePointSubscriber const&) = delete;
  ClosePointSubscriber& operator=(ClosePointSubscriber const&) = delete;

  static std::shared_ptr<ClosePointSubscriber> singleton() {
    static std::shared_ptr<ClosePointSubscriber> s_singleton{
        new ClosePointSubscriber()};
    return s_singleton;
  }

  const vector<laser_lines::msg::ClosestPointToLineSegment>&  getLineSements() const {
    return line_segments_;
  }

 private:
  vector<laser_lines::msg::ClosestPointToLineSegment> line_segments_;

  rclcpp::Subscription<laser_lines::msg::ClosestPointToLineSegmentList>::
      SharedPtr closest_line_subscriber_;

  void closestPointCallback(
      const laser_lines::msg::ClosestPointToLineSegmentList::SharedPtr msg);

  ClosePointSubscriber();
};