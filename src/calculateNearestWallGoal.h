#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "closePointSubscriber.h"
#include "rclcpp/rclcpp.hpp"

using namespace std;

class CalculateNearestWallGoal : public BT::SyncActionNode {
 public:
  CalculateNearestWallGoal(const std::string& name,
                           const BT::NodeConfiguration& cfg)
      : BT::SyncActionNode(name, cfg) {}

  static BT::PortsList providedPorts() {
    return {BT::OutputPort<geometry_msgs::msg::Pose>("goal")};
  }

  BT::NodeStatus tick() {
    const vector<laser_lines::msg::ClosestPointToLineSegment>& line_segments =
        ClosePointSubscriber::singleton()->getLineSements();
    float closest_wall_distance = 1'000'000.0;
    geometry_msgs::msg::Pose closest_wall_pose;

    for (size_t i = 0; i < line_segments.size(); i++) {
      if (line_segments[i].distance < closest_wall_distance) {
        closest_wall_distance = line_segments[i].distance;
        closest_wall_pose.position.x = line_segments[i].end[0];
        closest_wall_pose.position.y = line_segments[i].end[1];
        closest_wall_pose.position.z = 0.0;
        closest_wall_pose.orientation.w = 1.0;
        closest_wall_pose.orientation.x = 0.0;
        closest_wall_pose.orientation.y = 0.0;
        closest_wall_pose.orientation.z = 0.0;
      }
    }

    setOutput("goal", closest_wall_pose);
    std::cout << "[CalculateNearestWallGoal] x: "
              << closest_wall_pose.position.x
              << ", y: " << closest_wall_pose.position.y << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};